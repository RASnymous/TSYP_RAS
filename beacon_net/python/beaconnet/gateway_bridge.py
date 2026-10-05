#!/usr/bin/env python3
"""
gateway_bridge.py - LoRa gateway (or the simulator)  ->  Command Post dashboard
===============================================================================
Reads the gateway's JSON lines and forwards them to the dashboard:

    {"ev":"new"|"update"|"silent"|"revived"|"expired", record..., rx meta}
         -> POST /api/beacon          (contract fields + full LMB2 record)
    {"ev":"hb", ...}                  -> POST /api/network-health
    {"ev":"reject","reason":...}      -> counted; reported with the next health
                                         post and printed (possible spoofing)

Sources
    --serial /dev/ttyUSB0   real gateway on USB (needs `pip install pyserial`);
                            on connect it sends TIME <now> (and KEY if --key)
    --stdin                 anything piping gateway lines, e.g. the simulator:
                            ./bpsim --speed 20 | python3 gateway_bridge.py --stdin
    --sim="ARGS"            start the bundled simulator (host/bpsim, or
                            windows/bpsim.exe) with ARGS and read its output -
                            no shell pipe needed, works the same on Windows:
                            --sim="--speed 20 --beacons 30"
    --exec "CMD"            start any command and read its output
    (Windows serial ports are COM3, COM4, ...)

Only the standard library is needed for --stdin.

Examples
    python3 gateway_bridge.py --stdin --server http://localhost:3000
    python3 gateway_bridge.py --serial /dev/ttyUSB0 --key demo --server http://10.0.2.2:3000
    python3 gateway_bridge.py --stdin --dry-run           # print what would be sent
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beaconnet import proto  # noqa: E402
from beaconnet.dashboard import Poster, payload_from_gateway_line  # noqa: E402

RECORD_EVENTS = {"new", "update", "silent", "revived", "expired", "dump", "own"}


def open_serial(port: str, baud: int, key: str | None, net: int | None):
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("pyserial missing: pip install pyserial   (or use --stdin)")
    s = serial.Serial(port, baud, timeout=1)
    time.sleep(2.0)                       # ESP32 resets when the port opens
    s.reset_input_buffer()
    if key:
        hexkey = proto.DEMO_KEY.hex() if key == "demo" else key
        s.write(f"KEY {hexkey}\n".encode())
    if net is not None:
        s.write(f"NET {net}\n".encode())
    s.write(f"TIME {int(time.time())}\n".encode())
    s.write(b"STATUS\n")
    return s


def find_simulator() -> str:
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if os.name == "nt":
        cands = [os.path.join(root, "host", "bpsim.exe"), os.path.join(root, "windows", "bpsim.exe")]
    else:
        cands = [os.path.join(root, "host", "bpsim")]
    for c in cands:
        if os.path.isfile(c):
            return c
    sys.exit("simulator not found: run  make -C host  (Linux/WSL) or use the windows/ folder")


def lines_from(src):
    if hasattr(src, "readline") and not hasattr(src, "buffer"):      # pyserial
        while True:
            raw = src.readline()
            if raw:
                yield raw.decode("utf-8", "replace")
    else:
        for line in src:
            yield line


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--serial", metavar="PORT")
    g.add_argument("--stdin", action="store_true")
    g.add_argument("--sim", metavar="ARGS", help='run the bundled simulator with ARGS, e.g. --sim="--speed 20"')
    g.add_argument("--exec", dest="exec_cmd", metavar="CMD", help="run CMD and read its output")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--server", default="http://localhost:3000")
    ap.add_argument("--gw", default=None, help="gateway id shown on the dashboard (default: from hb, else gw1)")
    ap.add_argument("--key", help="32 hex chars, or 'demo' (serial only: provisions the gateway)")
    ap.add_argument("--net", type=int, help="network id (serial only)")
    ap.add_argument("--dry-run", action="store_true", help="print payloads instead of POSTing")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    say = (lambda *m: None) if a.quiet else (lambda *m: print(*m, file=sys.stderr, flush=True))
    poster = None if a.dry_run else Poster(a.server, log=say)
    gw = a.gw or "gw1"
    rejects: dict = {}
    last_known = {}
    time_scale = 1.0          # the simulator can run faster than real time (hb.sim_speed)
    proc = None
    if a.serial:
        src = open_serial(a.serial, a.baud, a.key, a.net)
    elif a.exec_cmd or a.sim is not None:
        if a.sim is not None:
            args = [find_simulator()] + shlex.split(a.sim, posix=os.name != "nt")
        else:
            args = a.exec_cmd if os.name == "nt" else shlex.split(a.exec_cmd)
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, text=True, bufsize=1)
        src = proc.stdout
    else:
        src = sys.stdin

    def send(path, body):
        if a.dry_run:
            print(path, json.dumps(body), flush=True)
        else:
            poster.post(path, body)

    origin = f"serial {a.serial}" if a.serial else ("simulator" if a.sim is not None else
                                                     (f"exec {a.exec_cmd}" if a.exec_cmd else "stdin"))
    say(f"[bridge] {origin} -> {a.server}{' (dry run)' if a.dry_run else ''}")
    try:
        for line in lines_from(src):
            line = line.strip()
            if not line.startswith("{"):
                if line and not a.quiet and (line.startswith("OK") or line.startswith("ERR")):
                    say(f"[gateway] {line}")
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            ev = d.get("ev")
            if ev == "hb":
                if not a.gw and d.get("gw"):
                    gw = d["gw"]
                if d.get("sim_speed"):
                    time_scale = float(d["sim_speed"])
                send("/api/network-health", {
                    "gateway_id": gw, "status": "online", "timestamp": d.get("ts", time.time()),
                    "rx_ok": d.get("rx_ok"), "rx_bad": d.get("rx_bad"), "known": d.get("known"),
                    "digests": d.get("digests"), "duty": d.get("duty"),
                    "rejected": sum(rejects.values()), "reject_reasons": dict(rejects),
                })
            elif ev == "reject":
                r = d.get("reason", "unknown")
                rejects[r] = rejects.get(r, 0) + 1
                say(f"[bridge] rejected frame: {r} (total {sum(rejects.values())}) - forged/tampered/foreign?")
            elif ev in RECORD_EVENTS and "beacon_id" in d:
                body = payload_from_gateway_line(d, gw)
                body["time_scale"] = time_scale      # dashboard ages records this much faster
                if body["lat"] is None or body["lon"] is None:
                    continue
                send("/api/beacon", body)
                bid = d["beacon_id"]
                if not a.quiet and (ev != "dump" or last_known.get(bid) != d.get("seq")):
                    tag = {"silent": "LOST?", "revived": "back", "expired": "aged out"}.get(ev, ev)
                    nxt = d.get("next") or {}
                    say(f"[bridge] #{bid:<3} {d.get('kind', '?'):<11} {tag:<8} seq {d.get('seq')} "
                        f"sev {d.get('severity')} conf {body['eff_conf']:.2f} hops {d.get('hops')}"
                        + (f" -> next #{nxt.get('id')}" if nxt else ""))
                last_known[bid] = d.get("seq")
    except KeyboardInterrupt:
        pass
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
        if poster:
            poster.flush()
            say(f"[bridge] posted {poster.sent}, failed {poster.failed}")


if __name__ == "__main__":
    main()
