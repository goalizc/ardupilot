#!/usr/bin/env python3
"""Fault-injection drill for the SHOW mode (single SITL instance).

Brings one vehicle into a SHOW performance, then injects a fault and
records how the vehicle behaves, so we can judge whether the behaviour
is "field-acceptable" (safe + explainable).

Scenarios:
  A  GPS loss mid-performance: SIM_GPS1_ENABLE=0  (expect EKF failsafe
     -> LAND mode fallback)
  C  Show file corrupted mid-performance: truncate show/show.bin before
     a still-needed block is read.  The reader fills the standby block as
     the playing block starts, so with 10Hz frames (--frame-ms 100, one
     window = 12.8s) a 40s show spans 4 blocks; block 3 is read at the
     block-2 start (t=12.8s), so injecting just before that hits the next
     refill:
     --scenario C --duration 40 --frame-ms 100 --inject-delay 12.3
  D  GCS disconnect mid-performance: drop the mavlink link entirely
     (expect the show completes autonomously -> RTL -> landed)

Usage:
  fault_inject.py --scenario A|C|D [--duration 20] [--out DIR]

Evidence: printed STATUSTEXT lines + DF log under out/inst0/logs/*.BIN
(parse with analyze.py / mavlogdump).
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "modules",
                                "mavlink", "pymavlink"))
from pymavlink import mavutil  # noqa: E402
from run_multi import Instance, ROOT  # noqa: E402

SCENARIOS = ('A', 'C', 'D')


def collect(inst, seconds, interesting=('Show', 'EKF', 'Land', 'PreArm',
                                        'Failsafe', 'DISARM', 'disarm')):
    """Print statustext lines for `seconds`, returning when a line matching
    `stop_on` (if given) appears."""
    t0 = time.time()
    lines = []
    while time.time() - t0 < seconds:
        m = inst.conn.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
        if m is None:
            continue
        if any(k in m.text for k in interesting):
            print(f"  [ST] {m.text}", flush=True)
            lines.append(m.text)
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True, choices=SCENARIOS)
    ap.add_argument('--duration', type=int, default=20)
    ap.add_argument('--frame-ms', type=int, default=1000,
                    help='position-frame spacing; 100 = 10Hz.  With 10Hz '
                         'frames the 128-frame window holds 12.8s, so '
                         'scenario C needs only a ~40s show.')
    ap.add_argument('--inject-delay', type=float, default=3.0,
                    help='seconds after performing before the injection')
    ap.add_argument('--observe', type=float, default=120,
                    help='seconds to watch after the injection')
    ap.add_argument('--out', default=os.path.join(ROOT, 'sitl', 'fault_run'))
    args = ap.parse_args()

    workdir = os.path.join(args.out, 'inst0')
    os.makedirs(workdir, exist_ok=True)
    start_time_utc = int(time.time())
    inst = Instance(0, workdir, start_time_utc, args.duration, 1.0,
                    frame_ms=args.frame_ms)
    print(f"[{args.scenario}] starting instance (show {args.duration}s)...", flush=True)
    try:
        inst.start()
        # bind confirmation: a NEW instance must own the port.  If a stale
        # instance from a previous run still listens, the new process dies
        # at bind and we must not connect to the stale one (its params are
        # indistinguishable in the connect() health check).
        time.sleep(2)
        if inst.proc.poll() is not None:
            raise SystemExit(
                f"[{args.scenario}] instance died at startup - port "
                f"{inst.port} is held by a stale instance.  Kill stale "
                "arducopter processes and rerun.")
        # wait for the TCP port then connect
        deadline = time.time() + 60
        while time.time() < deadline:
            try:
                import pymavlink.mavutil as mavutil
                mavutil.mavlink_connection(f"tcp:127.0.0.1:{inst.port}", timeout=1).close()
                break
            except Exception:
                time.sleep(0.2)
        inst.connect()
        inst.wait_gps_fix()
        print("[*] GPS fix", flush=True)

        # load the show file (autoload usually done; reload to be sure)
        inst.wait_text("Show loaded", timeout=30,
                       action=lambda: inst.conn.mav.command_long_send(
                           1, 1, mavutil.mavlink.MAV_CMD_USER_1, 0,
                           0, 0, 0, 0, 0, 0, 0))
        print("[*] show loaded", flush=True)

        # schedule the start a short while ahead
        t0 = (inst.gps_tow_sec() + 20) % 604800
        g = inst.conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)
        params = {
            "SHOW_START_TIME": t0,
            "SHOW_START_MSEC": 0,
            "SHOW_TAKEOFF_ALT": 5,
            "SHOW_POST_ACTION": 2,       # RTL (fallback only)
            "DISARM_DELAY": 0,
            "FS_THR_ENABLE": 0,
            "SHOW_AUTH": 1,
            "SHOW_VEL_FF_GAIN": 1.0,
            "SHOW_MAX_XY_ERR": 3.0,
            "SHOW_MAX_Z_ERR": 3.0,
            "SHOW_ORIGIN_LAT": g.lat,
            "SHOW_ORIGIN_LNG": g.lon,
            "SHOW_ORIGIN_AMSL": 0,
            "SHOW_ORIENTATION": 0,
        }
        for name, value in params.items():
            inst.set_param(name, value)
        inst.arm()
        t_mode = time.time()
        while time.time() - t_mode < 90:
            inst.send_mode(31)
            try:
                inst.wait_text("Show stage: waiting", timeout=10)
                break
            except RuntimeError:
                time.sleep(3)
        else:
            raise RuntimeError("could not enter SHOW mode")
        print("[*] waiting for the performance...", flush=True)
        inst.wait_text("Show stage: performing", timeout=120)
        print("[*] PERFORMING - injecting after "
              f"{args.inject_delay}s...", flush=True)
        time.sleep(args.inject_delay)

        if args.scenario == 'A':
            print("[A] SIM_GPS1_ENABLE=0 (GPS loss)", flush=True)
            inst.set_param("SIM_GPS1_ENABLE", 0)
            print("[A] watching for EKF failsafe / landing...", flush=True)
            collect(inst, args.observe)
        elif args.scenario == 'C':
            # the windowed reader fills the standby block right when the
            # playing block starts, so a corrupting write only matters if it
            # lands before a block the show still needs is read.  With 10Hz
            # frames (--frame-ms 100) one window holds 12.8s, so a 40s show
            # spans 4 blocks; block 3 is read at the block-2 start
            # (t=12.8s), so injecting ~0.3s earlier makes that refill read
            # the truncated file and the show exhausts at t~=27s.
            path = os.path.join(workdir, 'show', 'show.bin')
            size = os.path.getsize(path)
            print(f"[C] truncating {path} ({size} -> {size//2} bytes)",
                  flush=True)
            with open(path, 'r+b') as f:
                f.truncate(size // 2)
            print("[C] watching for data-unavailable error...", flush=True)
            collect(inst, args.observe)
        elif args.scenario == 'D':
            print("[D] dropping the GCS link mid-performance", flush=True)
            # stop the GCS heartbeat and drop the link; the SITL process
            # keeps running and the show must complete on its own
            inst._hb_stop = True
            inst.conn.close()
            print(f"[D] link dropped; waiting {args.duration + 40}s for the "
                  "show to finish autonomously...", flush=True)
            time.sleep(args.duration + 40)
            # reconnect as a fresh GCS and read the final state
            conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{inst.port}",
                                              timeout=5)
            stop = [False]

            def hb():
                while not stop[0]:
                    try:
                        conn.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    except Exception:
                        pass
                    time.sleep(1)

            threading.Thread(target=hb, daemon=True).start()
            t0 = time.time()
            while time.time() - t0 < 30:
                m = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                if m is not None and m.get_srcSystem() == 1:
                    armed = bool(m.base_mode &
                                 mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    print(f"[D] post-show state: mode={m.custom_mode} "
                          f"armed={armed}", flush=True)
                    if m.custom_mode in (0, 6) and not armed:  # STABILIZE/RTL
                        break
            stop[0] = True
            conn.close()
            return
        print("[*] done; DF log under %s/logs/" % workdir, flush=True)
    finally:
        inst.stop()
        # make sure the SITL process really dies so the port is free for
        # the next run (stale instances on 5760 are the classic trap)
        try:
            inst.proc.wait(timeout=5)
        except Exception:
            inst.proc.kill()


if __name__ == '__main__':
    main()
