"""
Run the Outside Network Area.

    python -m ona --config ona_config.json --udp 0.0.0.0:47100 --command-post http://localhost:3000
    python -m ona --config ona_config.json --serial gw1=COM3 --serial gw2=COM4 --serial gw3=COM5

Gateway inputs (any mix):
    --serial GW=PORT   a real gateway board on USB (pyserial). The ONA sends it KEY and TIME,
                       reads its JSON lines, and writes "TX <hex>" to transmit a briefing.
    --udp HOST:PORT    JSON lines over UDP from simulators or the ROS link node; each line
                       carries "gw". Briefings go back to the sender as {"ev":"tx","frame":...}.
    --stdin            JSON lines on standard input (e.g. a simulator piped in).
    --exec "CMD"       start CMD and read its output.

Everything leaves through the persistent queue and the uplink (LTE, else satellite).
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import shlex
import socket
import subprocess
import sys
import threading
import time

from .core import OnaCore, OnaConfig
from .store import PersistentQueue
from .uplink import CommandPostHttp, LinkModel, Uplink

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ona_config.json')


class Sources:
    """Reader threads -> one queue of (default gw, line, receive time). Also the downlink paths."""

    def __init__(self):
        self.q = queue.Queue(maxsize=20000)
        self.udp_sock = None
        self.udp_peers = {}          # gw -> (host, port) last heard from
        self.serial_ports = {}       # gw -> serial.Serial
        self._stop = threading.Event()
        self.threads = []

    def _put(self, gw, line):
        try:
            self.q.put_nowait((gw, line, time.time()))
        except queue.Full:
            pass

    def add_udp(self, spec: str) -> None:
        host, port = spec.rsplit(':', 1) if ':' in spec else ('0.0.0.0', spec)   # "47100" = every interface
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, int(port)))
        s.settimeout(0.5)
        self.udp_sock = s

        def run():
            while not self._stop.is_set():
                try:
                    data, addr = s.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break
                for ln in data.decode('utf-8', 'replace').splitlines():
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        gw = json.loads(ln).get('gw')
                    except (json.JSONDecodeError, AttributeError):
                        gw = None
                    if gw:
                        self.udp_peers[gw] = addr
                    self._put(None, ln)
        self._start(run, 'udp')

    def add_stdin(self) -> None:
        def run():
            for ln in sys.stdin:
                self._put(None, ln)
        self._start(run, 'stdin')

    def add_exec(self, cmd: str) -> None:
        p = subprocess.Popen(shlex.split(cmd, posix=os.name != 'nt'), stdout=subprocess.PIPE, text=True, bufsize=1)

        def run():
            for ln in p.stdout:
                self._put(None, ln)
        self._start(run, 'exec')

    def add_serial(self, gw: str, port: str, key: bytes, net_id: int) -> None:
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit('pyserial is needed for --serial: pip install pyserial')
        ser = serial.Serial(port, 115200, timeout=0.5)
        self.serial_ports[gw] = ser
        time.sleep(1.5)                           # ESP32 resets when the port opens
        for cmd in (f'KEY {key.hex()}', f'NET {net_id}', f'NAME {gw}', f'TIME {int(time.time())}'):
            ser.write((cmd + '\n').encode())
            time.sleep(0.1)

        def run():
            buf = b''
            while not self._stop.is_set():
                try:
                    chunk = ser.read(512)
                except Exception:  # noqa: BLE001 (board unplugged)
                    time.sleep(1.0)
                    continue
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    ln, buf = buf.split(b'\n', 1)
                    self._put(gw, ln.decode('utf-8', 'replace'))
        self._start(run, f'serial-{gw}')

    def _start(self, fn, name):
        th = threading.Thread(target=fn, name=name, daemon=True)
        th.start()
        self.threads.append(th)

    def transmit(self, frames: list) -> int:
        """Send briefing frames out through every gateway we can reach."""
        n = 0
        for gw, ser in self.serial_ports.items():
            for f in frames:
                try:
                    ser.write(f'TX {f.hex()}\n'.encode())
                    n += 1
                except Exception:  # noqa: BLE001
                    pass
        if self.udp_sock is not None:
            for gw, addr in list(self.udp_peers.items()):
                for f in frames:
                    try:
                        self.udp_sock.sendto(json.dumps({'ev': 'tx', 'gw': gw, 'frame': f.hex()}).encode(), addr)
                        n += 1
                    except OSError:
                        pass
        return n

    def stop(self):
        self._stop.set()


def console_line(core: OnaCore, up: Uplink) -> str:
    st = core.status(up.state.value)
    gws = '  '.join(f"{g['gw']} {'OK ' if g['alive'] else '-- '}{g['lines']}"
                    f"{' QUARANTINED' if g['quarantined'] else ''}" for g in st['gateways'])
    r = st['records']
    rob = '  '.join(f"robot#{x['robot_id']} {x['role'] or ''} {x['consistency'] or ''}" for x in st['robots'])
    return (f"[ONA] link {st['link']['state']:9s} queue {st['link']['pending']:4d} | {gws} | records "
            f"{r['confirmed']} confirmed {r['unconfirmed']} unconfirmed {r['conflicts']} conflict | {rob}")


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(line_buffering=True)        # console lines show up at once, even in a log file
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser(prog='python -m ona', description='Living Map - Outside Network Area')
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='ONA config (gateways, anchor, key)')
    ap.add_argument('--serial', action='append', default=[], metavar='GW=PORT')
    ap.add_argument('--udp', action='append', default=[], metavar='HOST:PORT')
    ap.add_argument('--stdin', action='store_true')
    ap.add_argument('--exec', dest='exec_cmd', action='append', default=[])
    ap.add_argument('--command-post', default=None, help='dashboard URL (default: config, else localhost:3000)')
    ap.add_argument('--db', default='ona_queue.sqlite3', help='SQLite queue + audit file')
    ap.add_argument('--fresh', action='store_true', help='start with an empty queue (deletes the db file)')
    ap.add_argument('--lte-outage', default='', help='simulated LTE outages, seconds after start: 60-150,300-320')
    ap.add_argument('--no-lte', action='store_true')
    ap.add_argument('--no-sat', action='store_true')
    ap.add_argument('--strict', action='store_true', help='never show unconfirmed (1/3) records')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)

    with open(a.config, encoding='utf-8') as f:
        cfg_d = json.load(f)
    if a.strict:
        cfg_d['strict'] = True
    cfg = OnaConfig.from_dict(cfg_d)
    base = a.command_post or cfg_d.get('command_post', 'http://localhost:3000')
    if a.fresh and os.path.exists(a.db):
        os.remove(a.db)
    store = PersistentQueue(a.db)
    links = LinkModel(lte=not a.no_lte, sat=not a.no_sat, lte_outages=a.lte_outage)
    transport = CommandPostHttp(base, links)
    log = (lambda s: None) if a.quiet else (lambda s: print(s, flush=True))
    src = Sources()
    core = OnaCore(cfg, store, downlink=lambda frames: src.transmit(frames), log=log)
    up = Uplink(store, transport, audit=store.audit)
    up.start()

    for spec in a.serial:
        gw, port = spec.split('=', 1)
        src.add_serial(gw, port, cfg.key, cfg.net_id)
    for spec in a.udp:
        src.add_udp(spec)
    if a.stdin:
        src.add_stdin()
    for c in a.exec_cmd:
        src.add_exec(c)
    if not (a.serial or a.udp or a.stdin or a.exec_cmd):
        src.add_udp('0.0.0.0:47100')
        a.udp = ['0.0.0.0:47100']

    print('=' * 72)
    print(f' Living Map - Outside Network Area ({cfg.name})')
    print(f'   gateways : {", ".join(f"{g} ({la:.6f}, {lo:.6f})" for g, (la, lo, al) in cfg.gateways_gps.items())}')
    print(f'   vote     : a message counts when {cfg.quorum} of {len(cfg.gateways)} gateways agree'
          f'{" (strict)" if cfg.strict else ""}')
    print(f'   inputs   : {", ".join(a.serial + a.udp + (["stdin"] if a.stdin else []) + a.exec_cmd)}')
    print(f'   uplink   : {base}  LTE {"off" if a.no_lte else "on"}{" outages " + a.lte_outage if a.lte_outage else ""}'
          f', satellite {"off" if a.no_sat else "backup"}')
    print(f'   queue    : {os.path.abspath(a.db)}')
    print('=' * 72, flush=True)

    last_poll = last_console = 0.0
    try:
        while True:
            try:
                gw, line, t = src.q.get(timeout=0.1)
                core.handle_line(line, default_gw=gw)
                n = 0
                while n < 500:
                    gw, line, t = src.q.get_nowait()
                    core.handle_line(line, default_gw=gw)
                    n += 1
            except queue.Empty:
                pass
            core.tick(up.state.value)
            now = time.time()
            if now - last_poll > 2.0:
                last_poll = now
                m = transport.get('/api/mission-dispatch/latest')
                if m:
                    core.on_dashboard_mission(m)
            if not a.quiet and now - last_console > 5.0:
                last_console = now
                print(console_line(core, up), flush=True)
    except KeyboardInterrupt:
        print('\n[ONA] stopping; unsent messages stay in', os.path.abspath(a.db))
    finally:
        src.stop()
        up.stop()
    return 0
