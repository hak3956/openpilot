"""EV6 HUD patch v24: the phone draws and encodes the HUD, the comma only forwards it to the TURZX screen.

The Carrot Vision 3D app (EV6 build, settings › HUD로 보내기) renders its HUD page in the background, encodes it with the
phone's hardware H.264 encoder and connects here (TCP 7090). While its frames arrive, the cluster's main loop skips
its own rendering and encoding (main.py checks active()), and every phone frame goes through the cluster's own SPS
fix-ups and USB packetising (H264UsbPipeline.submit_external_access_unit). When the phone stops for more than a
second, the cluster draws the HUD again by itself.

Protocol (phone = client):
  comma -> phone  one JSON line: {"v":1,"ready":true,"w":1920,"h":480,"fps":20,"bitrate":4000000}
                  or {"v":1,"ready":false,"why":"..."} and the connection is closed (try again later)
  phone -> comma  frames: [u32 length, big endian][u8 flags, 1 = key frame][H.264 Annex-B access unit]

Off switch: touch /data/ev6_hud_phone_off  (rm to allow the phone again; checked on every new connection)
"""
from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time

PORT = 7090
ACTIVE_S = 1.0           # the cluster draws again after this long without a phone frame
MAX_FRAME = 4 << 20       # sanity limit for one access unit
OFF_FLAG = '/data/ev6_hud_phone_off'

_lock = threading.Lock()
_pipeline = None
_server = None
_last_rx = 0.0
_frames = 0
_conn: socket.socket | None = None


def _bitrate_bps(pipeline) -> int:
  """The cluster's own H.264 bitrate (e.g. "2300k", "4M", "auto") as bits per second."""
  text = str(getattr(pipeline, 'bitrate', 'auto') or 'auto').strip().lower()
  fps = max(1, int(getattr(pipeline, 'fps', 20)))
  try:
    if text.endswith('m'):
      return int(float(text[:-1]) * 1_000_000)
    if text.endswith('k'):
      return int(float(text[:-1]) * 1_000)
    if text not in ('', 'auto'):
      return int(float(text))
  except ValueError:
    pass
  # same rule as the cluster's "auto": 7 Mbps at 30 fps, scaled, 1..14 Mbps
  return int(max(1_000_000, min(14_000_000, fps * 7_000_000 / 30)))


def start(pipeline) -> None:
  """Called once the cluster's H.264 USB pipeline runs; (re)uses one listening thread per process."""
  global _pipeline, _server
  with _lock:
    _pipeline = pipeline
    if _server is None:
      _server = threading.Thread(target=_serve, name='ev6-hud-phone', daemon=True)
      _server.start()


def stop() -> None:
  global _pipeline
  with _lock:
    _pipeline = None
  _drop()


def active() -> bool:
  """True while phone frames arrive: the cluster then skips drawing and encoding its own HUD frames."""
  return _pipeline is not None and time.monotonic() - _last_rx < ACTIVE_S


def _drop() -> None:
  global _conn
  c = _conn
  _conn = None
  if c is not None:
    try:
      c.close()
    except OSError:
      pass


def _serve() -> None:
  while True:
    try:
      srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      srv.bind(('0.0.0.0', PORT))
      srv.listen(2)
      print(f'EV6 HUD phone: listening on {PORT}', flush=True)
      while True:
        conn, addr = srv.accept()
        _drop()  # a new phone connection replaces the old one
        threading.Thread(target=_handle, args=(conn, addr), name='ev6-hud-phone-rx', daemon=True).start()
    except OSError as e:
      print(f'EV6 HUD phone: server error {e}; retrying', flush=True)
      time.sleep(5)


def _recv_exact(conn: socket.socket, n: int) -> bytes:
  buf = bytearray()
  while len(buf) < n:
    part = conn.recv(n - len(buf))
    if not part:
      raise ConnectionError('closed')
    buf += part
  return bytes(buf)


def _handle(conn: socket.socket, addr) -> None:
  global _conn, _last_rx, _frames
  pipeline = _pipeline
  try:
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if os.path.exists(OFF_FLAG):
      conn.sendall(json.dumps({'v': 1, 'ready': False, 'why': '콤마에서 꺼 둠 (/data/ev6_hud_phone_off)'}).encode() + b'\n')
      return
    if pipeline is None:
      conn.sendall(json.dumps({'v': 1, 'ready': False, 'why': 'HUD가 연결되지 않았습니다'}).encode() + b'\n')
      return
    hello = {'v': 1, 'ready': True, 'w': int(pipeline.width), 'h': int(pipeline.height), 'fps': int(pipeline.fps),
             'bitrate': _bitrate_bps(pipeline)}
    conn.sendall(json.dumps(hello).encode() + b'\n')
    _conn = conn
    print(f'EV6 HUD phone: {addr[0]} streaming {hello["w"]}x{hello["h"]} @ {hello["fps"]} fps', flush=True)
    n0, t0 = 0, time.monotonic()
    while _conn is conn:
      length, flags = struct.unpack('>IB', _recv_exact(conn, 5))
      if length == 0 or length > MAX_FRAME:
        raise ValueError(f'bad frame length {length}')
      unit = _recv_exact(conn, length)
      if _pipeline is not pipeline:  # the cluster restarted its pipeline: the phone reconnects and gets a new hello
        break
      pipeline.submit_external_access_unit(unit)
      _last_rx = time.monotonic()
      _frames += 1
      if _last_rx - t0 >= 10.0:
        print(f'EV6 HUD phone: {(_frames - n0) / (_last_rx - t0):.1f} fps from {addr[0]}', flush=True)
        n0, t0 = _frames, _last_rx
  except Exception as e:  # noqa: BLE001 - any error ends this connection; the phone reconnects
    print(f'EV6 HUD phone: {addr[0]} ended: {type(e).__name__}: {e}', flush=True)
  finally:
    if _conn is conn:
      _conn = None
    try:
      conn.close()
    except OSError:
      pass
