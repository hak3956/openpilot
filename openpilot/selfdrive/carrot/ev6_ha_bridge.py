#!/usr/bin/env python3
"""EV6 HUD patch v20: send the car's status and finished trips to Home Assistant (webhook over HTTPS).

Installed as openpilot/selfdrive/carrot/ev6_ha_bridge.py and started by the manager (always_run).
Does nothing until /data/ev6_ha.json exists:

  {"url": "https://<home assistant>/api/webhook/<webhook id>"}

optional keys: "interval_s" (driving, default 15), "parked_interval_s" (default 120).
- status  -> POST <url>        every interval while driving, every parked interval otherwise
- trips   -> POST <url>_trip   once when the car is switched off; kept in /data/ev6_ha_queue.jsonl until sent
- state of charge comes from CAN (message 762 byte 15 = SoC x 2, same as the parked HUD screen, v18)
- `touch /data/ev6_ha_off` stops sending (checked every few seconds)
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import urllib.request

CONFIG_PATH = "/data/ev6_ha.json"
OFF_FLAG = "/data/ev6_ha_off"
STATE_PATH = "/data/ev6_ha_state.json"  # total distance and the trip in progress (survives restarts)
QUEUE_PATH = "/data/ev6_ha_queue.jsonl"  # finished trips not yet delivered

DEFAULT_INTERVAL_S = 15.0
DEFAULT_PARKED_INTERVAL_S = 120.0
HTTP_TIMEOUT_S = 8.0
QUEUE_MAX_TRIPS = 200
STATE_SAVE_S = 60.0
TRIP_RESUME_S = 600.0  # a restart within 10 min of the last save continues the same trip
TRIP_MIN_KM = 0.1
PATH_SPACING_M = 50.0
PATH_MAX_POINTS = 200  # keeps the trip attributes well under Home Assistant's 16 KB recorder limit

SOC_CAN_ADDR = 762  # 0x2FA, bus 0, byte 15 = SoC x 2 (0.5 % steps); see Ev6Battery in cluster_renderer.py
SOC_CAN_BUS = 0
SOC_BYTE = 15
SOC_REFRESH_S = 30.0
SOC_MAX_AGE_S = 600.0

KPH = 3.6
ENGAGED_MIN_SPEED = 1.0  # m/s; standing still does not count towards the auto-driving share


def soc_from_can(frames) -> float | None:
  for frame in frames:
    if frame.address == SOC_CAN_ADDR and frame.src == SOC_CAN_BUS and len(frame.dat) > SOC_BYTE:
      soc = frame.dat[SOC_BYTE] * 0.5
      if 0.0 <= soc <= 100.0:
        return soc
  return None


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
  lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
  return 2 * 6371000.0 * math.asin(math.sqrt(h))


def load_json(path: str, default):
  try:
    with open(path, encoding="utf-8") as f:
      return json.load(f)
  except (OSError, ValueError):
    return default


def save_json(path: str, data) -> None:
  tmp = path + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False)
  os.replace(tmp, path)


class Trip:
  """Accumulates one drive (ignition on -> off) from speed, GPS and the engaged state."""

  def __init__(self, start_ts: float, soc: float | None, pos: tuple[float, float] | None):
    self.start_ts = start_ts
    self.end_ts = start_ts
    self.distance_m = 0.0
    self.moving_s = 0.0
    self.engaged_m = 0.0
    self.max_speed = 0.0
    self.soc_start = soc
    self.soc_end = soc
    self.start_pos = pos
    self.end_pos = pos
    self.path: list[list[float]] = [[round(pos[0], 5), round(pos[1], 5)]] if pos else []
    self._spacing = PATH_SPACING_M

  def update(self, now: float, dt: float, speed: float, engaged: bool, soc: float | None,
             pos: tuple[float, float] | None) -> None:
    self.end_ts = now
    step = max(speed, 0.0) * dt
    self.distance_m += step
    if speed > 0.3:
      self.moving_s += dt
    if engaged and speed > ENGAGED_MIN_SPEED:
      self.engaged_m += step
    self.max_speed = max(self.max_speed, speed)
    if soc is not None:
      if self.soc_start is None:
        self.soc_start = soc
      self.soc_end = soc
    if pos is not None:
      if self.start_pos is None:
        self.start_pos = pos
      self.end_pos = pos
      if not self.path or haversine_m((self.path[-1][0], self.path[-1][1]), pos) >= self._spacing:
        self.path.append([round(pos[0], 5), round(pos[1], 5)])
        if len(self.path) > PATH_MAX_POINTS:
          self.path = self.path[::2] + ([self.path[-1]] if len(self.path) % 2 == 0 else [])
          self._spacing *= 2

  def report(self) -> dict:
    km = self.distance_m / 1000.0
    duration = max(self.end_ts - self.start_ts, 0.0)
    soc_used = None
    if self.soc_start is not None and self.soc_end is not None:
      soc_used = round(self.soc_start - self.soc_end, 1)
    return {
      "type": "trip",
      "start_ts": round(self.start_ts),
      "end_ts": round(self.end_ts),
      "distance_km": round(km, 2),
      "duration_s": round(duration),
      "avg_kph": round(km / (self.moving_s / 3600.0), 1) if self.moving_s > 0 else 0.0,
      "max_kph": round(self.max_speed * KPH),
      "auto_pct": round(100.0 * self.engaged_m / self.distance_m) if self.distance_m > 0 else 0,
      "soc_start": self.soc_start,
      "soc_end": self.soc_end,
      "soc_used": soc_used,
      "start_lat": self.start_pos[0] if self.start_pos else None,
      "start_lon": self.start_pos[1] if self.start_pos else None,
      "end_lat": self.end_pos[0] if self.end_pos else None,
      "end_lon": self.end_pos[1] if self.end_pos else None,
      "path": self.path,
    }

  def to_dict(self) -> dict:
    return {k: v for k, v in self.__dict__.items()}

  @classmethod
  def from_dict(cls, d: dict) -> Trip:
    trip = cls.__new__(cls)
    trip.__dict__.update(d)
    trip.start_pos = tuple(trip.start_pos) if trip.start_pos else None
    trip.end_pos = tuple(trip.end_pos) if trip.end_pos else None
    return trip


class Sender:
  """Posts in its own thread so a slow hotspot never stalls the distance integration."""

  def __init__(self):
    self._lock = threading.Lock()
    self._wake = threading.Event()
    self._status: dict | None = None
    self.url = ""
    self.last_ok = 0.0
    threading.Thread(target=self._run, name="ev6-ha-send", daemon=True).start()

  def send_status(self, url: str, payload: dict) -> None:
    with self._lock:
      self.url, self._status = url, payload
    self._wake.set()

  def queue_trip(self, report: dict) -> None:
    with self._lock:
      trips = self._read_queue()[-(QUEUE_MAX_TRIPS - 1):] + [report]
      with open(QUEUE_PATH + ".tmp", "w", encoding="utf-8") as f:
        f.writelines(json.dumps(t, ensure_ascii=False) + "\n" for t in trips)
      os.replace(QUEUE_PATH + ".tmp", QUEUE_PATH)
    self._wake.set()

  @staticmethod
  def _read_queue() -> list[dict]:
    try:
      with open(QUEUE_PATH, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]
    except (OSError, ValueError):
      return []

  @staticmethod
  def _post(url: str, payload: dict) -> None:
    req = urllib.request.Request(url, data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
      resp.read()

  def _run(self) -> None:
    while True:
      self._wake.wait(30.0)
      self._wake.clear()
      with self._lock:
        url, status, self._status = self.url, self._status, None
      if not url:
        continue
      try:
        while True:  # oldest trip first; each one leaves the queue only after it was delivered
          with self._lock:
            trips = self._read_queue()
          if not trips:
            break
          self._post(url + "_trip", trips[0])
          with self._lock:
            rest = self._read_queue()[1:]
            with open(QUEUE_PATH + ".tmp", "w", encoding="utf-8") as f:
              f.writelines(json.dumps(t, ensure_ascii=False) + "\n" for t in rest)
            os.replace(QUEUE_PATH + ".tmp", QUEUE_PATH)
        if status is not None:
          self._post(url, status)
        self.last_ok = time.time()
      except Exception as exc:
        print(f"ev6_ha_bridge: send failed: {exc}", flush=True)


class SocReader:
  """Latest state of charge from the 'can' stream; same sampling as the parked HUD screen."""

  def __init__(self):
    self._lock = threading.Lock()
    self._soc: float | None = None
    self._soc_time = 0.0
    threading.Thread(target=self._run, name="ev6-ha-soc", daemon=True).start()

  def get(self) -> tuple[float | None, float | None]:
    with self._lock:
      if self._soc is None:
        return None, None
      age = time.monotonic() - self._soc_time
      return (self._soc, age) if age <= SOC_MAX_AGE_S else (None, None)

  def _run(self) -> None:
    import openpilot.cereal.messaging as messaging

    sock = None
    while True:
      soc = None
      try:
        if sock is None:
          sock = messaging.sub_sock("can", conflate=True, timeout=200)
        deadline = time.monotonic() + 3.0  # the message comes at 10 Hz, one in ~10 'can' batches
        while soc is None and time.monotonic() < deadline:
          msg = messaging.recv_one(sock)
          if msg is not None:
            soc = soc_from_can(msg.can)
        if soc is not None:
          with self._lock:
            self._soc, self._soc_time = soc, time.monotonic()
      except Exception as exc:
        print(f"ev6_ha_bridge: CAN read failed: {exc}", flush=True)
        sock = None
      time.sleep(SOC_REFRESH_S if soc is not None else 5.0)


def last_gps_position() -> tuple[float, float] | None:
  try:
    from openpilot.common.params import Params
    for params in (Params("/dev/shm/params"), Params()):
      raw = params.get("LastGPSPosition")
      if raw:
        pos = json.loads(raw)
        return float(pos["latitude"]), float(pos["longitude"])
  except Exception:
    pass
  return None


def main() -> None:
  import openpilot.cereal.messaging as messaging

  sm = messaging.SubMaster(["deviceState", "carState", "selfdriveState", "peripheralState", "pandaStates",
                            "gpsLocationExternal", "gpsLocation"])
  sender = Sender()
  soc_reader = SocReader()

  saved = load_json(STATE_PATH, {})
  total_m = float(saved.get("total_m", 0.0))
  trip = None
  if saved.get("trip") and time.time() - float(saved.get("saved_ts", 0)) < TRIP_RESUME_S:
    trip = Trip.from_dict(saved["trip"])
  elif saved.get("trip"):
    sender.queue_trip(Trip.from_dict(saved["trip"]).report())  # the car was switched off while we were down
  last_save = last_send = 0.0
  last_tick = last_param_read = time.monotonic()
  was_started = trip is not None
  last_pos = last_gps_position()

  while True:
    sm.update(1000)
    if sm.recv_frame["deviceState"] == 0:
      continue  # 'started' is unknown until the first deviceState; a resumed trip must not end here
    tick = time.monotonic()
    dt = min(tick - last_tick, 5.0)
    last_tick = tick
    now = time.time()

    config = load_json(CONFIG_PATH, None) if not os.path.exists(OFF_FLAG) else None
    url = str(config.get("url", "")).strip() if isinstance(config, dict) else ""

    started = bool(sm["deviceState"].started)
    soc, soc_age = soc_reader.get()

    pos = None
    for gps_name in ("gpsLocationExternal", "gpsLocation"):
      gps = sm[gps_name]
      if sm.recv_frame[gps_name] > 0 and started and gps.hasFix and tick - sm.recv_time[gps_name] < 5.0:
        pos = (gps.latitude, gps.longitude)
        break
    if pos is not None:
      last_pos = pos
    elif not started and tick - last_param_read > 60.0:
      last_param_read = tick
      last_pos = last_gps_position() or last_pos

    cs = sm["carState"]
    speed = float(cs.vEgo) if started else 0.0
    engaged = bool(sm["selfdriveState"].enabled) if started else False

    if started and not was_started:
      trip = Trip(now, soc, pos)
    if started and trip is not None:
      trip.update(now, dt, speed, engaged, soc, pos)
      total_m += max(speed, 0.0) * dt
    if not started and was_started and trip is not None:
      if trip.distance_m >= TRIP_MIN_KM * 1000.0:
        sender.queue_trip(trip.report())
      trip = None
      last_send = 0.0  # report "parked" right away
    was_started = started

    if now - last_save > STATE_SAVE_S or (trip is None and saved.get("trip")):
      saved = {"total_m": total_m, "trip": trip.to_dict() if trip else None, "saved_ts": now}
      save_json(STATE_PATH, saved)
      last_save = now

    interval = DEFAULT_INTERVAL_S if started else DEFAULT_PARKED_INTERVAL_S
    if isinstance(config, dict):
      interval = float(config.get("interval_s" if started else "parked_interval_s", interval))
    if not url or tick - last_send < interval:
      continue
    last_send = tick

    if sm.recv_frame["peripheralState"] > 0 and sm["peripheralState"].voltage > 0:
      v12 = sm["peripheralState"].voltage / 1000.0
    elif sm.recv_frame["pandaStates"] > 0 and len(sm["pandaStates"]) > 0:
      v12 = sm["pandaStates"][0].voltage / 1000.0
    else:
      v12 = None
    gear = str(cs.gearShifter) if started else "off"
    tpms = cs.tpms if started else None
    payload = {
      "type": "status",
      "ts": round(now),
      "started": started,
      "status": ("주행중" if speed > 0.5 else "정차중") if started else "주차중",
      "gear": gear,
      "speed_kph": round(speed * KPH, 1),
      "engaged": engaged,
      "soc": soc,
      "soc_age_s": round(soc_age) if soc_age is not None else None,
      "v12": round(v12, 2) if v12 else None,
      "lat": round(last_pos[0], 6) if last_pos else None,
      "lon": round(last_pos[1], 6) if last_pos else None,
      "gps_fix": pos is not None,
      "total_km": round(total_m / 1000.0, 2),
      "trip_km": round(trip.distance_m / 1000.0, 2) if trip else None,
      "trip_s": round(trip.end_ts - trip.start_ts) if trip else None,
      "tpms_fl": round(tpms.fl, 1) if tpms and tpms.fl > 0 else None,
      "tpms_fr": round(tpms.fr, 1) if tpms and tpms.fr > 0 else None,
      "tpms_rl": round(tpms.rl, 1) if tpms and tpms.rl > 0 else None,
      "tpms_rr": round(tpms.rr, 1) if tpms and tpms.rr > 0 else None,
    }
    sender.send_status(url, payload)


if __name__ == "__main__":
  main()
