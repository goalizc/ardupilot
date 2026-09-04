#!/usr/bin/env python3
"""Run N SITL Copter instances through the same drone show and collect logs.

Each instance gets its own working directory (with its own copy of the
show file and its own dataflash logs), the same SHOW_START_TIME (computed
from the GPS time-of-week once every instance has a fix, plus a margin),
then arms, enters SHOW mode (31) and rides the full flow at 1x speed.

SITL GPS time is anchored to the host wall clock, so all instances share
the same GPS time base (verified empirically to 0.000s difference).

Usage:
  run_multi.py [--count N] [--duration S] [--speedup X] [--out DIR]
"""
import argparse
import os
import shutil
import signal
import subprocess
import threading
import sys
import time

from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
COPTER = os.path.join(ROOT, "build/sitl/bin/arducopter")
HOME = "-35.363261,149.165230,584,353"
UNIX_OFFSET_MSEC = 17000 * 86400 + 520 * 604800 * 1000 - 18000  # from AP_GPS.h


class Instance:
    def __init__(self, idx, workdir, start_time, duration_s, speedup, frame_ms=1000):
        self.idx = idx
        self.workdir = workdir
        os.makedirs(workdir, exist_ok=True)
        showdir = os.path.join(workdir, "show")
        os.makedirs(showdir, exist_ok=True)
        self.show_path = os.path.join(showdir, "show.bin")
        # generate the demo show file for this instance
        gen = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gen_show.py")
        cmd = [sys.executable, gen, self.show_path, str(duration_s)]
        if frame_ms != 1000:
            cmd += ['--frame-ms', str(frame_ms)]
        subprocess.check_call(cmd)
        self.port = 5760 + 10 * idx   # SITL adds 10*instance to all port numbers
        self.conn = None
        self.proc = None
        self.start_time = start_time
        self.speedup = speedup

    def start(self):
        log = open(os.path.join(self.workdir, "sitl.out"), "w")
        cmd = [COPTER, f"-I{self.idx}", "-M", "quad", "-O", HOME,
               f"--start-time={self.start_time}"]
        if self.speedup != 1:
            cmd += ["--speedup", str(self.speedup)]
        self.proc = subprocess.Popen(cmd, cwd=self.workdir,
                                     stdout=log, stderr=subprocess.STDOUT)
        return self

    def connect(self, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                self.conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.port}", timeout=5)
                # GCS heartbeat: the SITL link only starts streaming GPS /
                # answering streams while a ground station is present
                self._hb_stop = False
                self._hb = threading.Thread(target=self._heartbeat, daemon=True)
                self._hb.start()
                # let a few heartbeats reach the autopilot before requesting
                # streams (SITL only streams while a GCS is present)
                time.sleep(2)
                # ask for GPS and status streams (SITL does not send them by default)
                self.conn.mav.command_long_send(
                    1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200000, 0, 0, 0, 0, 0)
                # param channel must answer: proves this is a healthy instance
                # (a stale simulator on the same port may still stream GPS)
                self.conn.mav.param_request_read_send(1, 1, b'SHOW_AUTH', -1)
                t1 = time.time()
                while time.time() - t1 < 10:
                    m = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
                    if m is not None:
                        return self
                # unhealthy connection: drop and retry the port
                self.conn.close()
                time.sleep(1)
            except Exception:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"instance {self.idx} died at startup")
                time.sleep(1)
        raise RuntimeError(f"instance {self.idx}: no healthy mavlink on port {self.port}")

    def _heartbeat(self):
        while not getattr(self, '_hb_stop', False):
            try:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                             0, 0, 0)
            except Exception:
                pass
            time.sleep(1)

    def wait_gps_fix(self, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if m is not None and m.fix_type >= 3:
                return
        raise RuntimeError(f"instance {self.idx}: no GPS fix")

    def gps_tow_sec(self, timeout=30):
        """Current GPS time-of-week in seconds, read via SYSTEM_TIME."""
        self.conn.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 100000, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.conn.recv_match(type='SYSTEM_TIME', blocking=True, timeout=5)
            if m is not None and m.time_unix_usec > 0:
                tow_ms = ((m.time_unix_usec // 1000) - UNIX_OFFSET_MSEC) % (604800 * 1000)
                return tow_ms // 1000
        raise RuntimeError(f"instance {self.idx}: no SYSTEM_TIME")

    def set_param(self, name, value, timeout=20):
        self.conn.mav.param_set_send(1, 1, name.encode(), float(value),
                                     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if m is not None and m.param_id.strip('\x00') == name:
                if abs(m.param_value - value) > 0.001 * max(1, abs(value)):
                    raise RuntimeError(
                        f"instance {self.idx}: param {name} set to {m.param_value}, want {value}")
                return
        raise RuntimeError(f"instance {self.idx}: param {name} not confirmed")

    def wait_text(self, fragment, timeout, action=None):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if action is not None:
                action()
                action = None
            m = self.conn.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
            if m is not None:
                if 'Show' in m.text or 'PreArm' in m.text or 'Arm' in m.text or 'EKF' in m.text:
                    print(f"  [inst {self.idx} ST] {m.text}", flush=True)
                if fragment in m.text:
                    return
        raise RuntimeError(f"instance {self.idx}: timeout waiting for text '{fragment}'")

    def send_mode(self, mode):
        self.conn.mav.set_mode_send(1,
                                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                    mode)

    def arm(self, timeout=90):
        """Arm, retrying until the EKF/position checks pass and the
        vehicle reports armed (autotest's wait_ready_to_arm equivalent)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.conn.mav.command_long_send(1, 1,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                            1, 0, 0, 0, 0, 0, 0)
            t1 = time.time()
            while time.time() - t1 < 4:
                hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
                if hb is not None and hb.get_srcSystem() == 1:
                    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        return
            time.sleep(2)
        raise RuntimeError(f"instance {self.idx}: could not arm")

    def stop(self):
        self._hb_stop = True
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=10)
    ap.add_argument('--duration', type=int, default=60)
    ap.add_argument('--speedup', type=float, default=1.0)
    ap.add_argument('--frame-ms', type=int, default=1000)
    ap.add_argument('--out', default=os.path.join(ROOT, "sitl", "show_multi_run"))
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    start_time_utc = int(time.time())
    margin_s = 60          # seconds after the last GPS fix before T0
    instances = []

    try:
        # phase 1: start everything, then connect in parallel.  A SITL
        # instance anchors its simulated (GPS) clock when it is first
        # connected, so connecting serially would give each vehicle a
        # clock offset equal to the connect order delay.  Parallel
        # connects keep the formation on the same clock.
        print(f"starting {args.count} instances (speedup {args.speedup}, "
              f"show {args.duration}s)...", flush=True)
        for i in range(args.count):
            inst = Instance(i, os.path.join(args.out, f"inst{i}"),
                            start_time_utc, args.duration, args.speedup, args.frame_ms)
            inst.start()
            instances.append(inst)
        # wait until every TCP port is listening (startup), then connect
        # all instances at the same moment
        deadline = time.time() + 60
        for inst in instances:
            while time.time() < deadline:
                try:
                    mavutil.mavlink_connection(f"tcp:127.0.0.1:{inst.port}", timeout=1).close()
                    break
                except Exception:
                    time.sleep(0.2)
            else:
                raise RuntimeError(f"instance {inst.idx}: port {inst.port} never opened")

        errors = {}
        def bring_up(inst):
            try:
                inst.connect()
                inst.wait_gps_fix()
            except Exception as e:
                errors[inst.idx] = str(e)
        threads = [threading.Thread(target=bring_up, args=(inst,)) for inst in instances]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise RuntimeError(f"instance bring-up failed: {errors}")
        for inst in instances:
            print(f"instance {inst.idx}: GPS fix on port {inst.port}", flush=True)

        # phase 2: load the show file and compute a common T0
        for inst in instances:
            inst.wait_text("Show loaded", timeout=30,
                           action=lambda i=inst: i.conn.mav.command_long_send(
                               1, 1, mavutil.mavlink.MAV_CMD_USER_1, 0,
                               0, 0, 0, 0, 0, 0, 0))
            print(f"instance {inst.idx}: show loaded")
        max_tow = max(inst.gps_tow_sec() for inst in instances)
        t0 = (max_tow + margin_s) % 604800
        print(f"common SHOW_START_TIME T0 = {t0} (current max ToW {max_tow})")

        # phase 3: configure, arm, enter SHOW - in parallel so every
        # vehicle is waiting for T0 well before the start
        def configure(inst):
            # read the current position from the (already-streaming) GPS
            # message; mavutil's location() would block waiting for streams
            # we do not request (VFR_HUD / GLOBAL_POSITION_INT)
            g = inst.conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)
            if g is None:
                raise RuntimeError(f"instance {inst.idx}: no GPS position")
            params = {
                "SHOW_START_TIME": t0,
                "SHOW_START_MSEC": 0,
                "SHOW_TAKEOFF_ALT": 5,
                "SHOW_POST_ACTION": 2,       # RTL
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
            # SHOW mode requires a valid position estimate; retry until the
            # EKF has converged and the mode switch is accepted
            t_mode = time.time()
            while time.time() - t_mode < 90:
                inst.send_mode(31)          # SHOW
                try:
                    inst.wait_text("Show stage: waiting", timeout=10)
                    break
                except RuntimeError:
                    time.sleep(3)
            else:
                raise RuntimeError(f"instance {inst.idx}: could not enter SHOW mode")

        errors = {}
        def setup(inst):
            try:
                configure(inst)
            except Exception as e:
                errors[inst.idx] = str(e)
        threads = [threading.Thread(target=setup, args=(inst,)) for inst in instances]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise RuntimeError(f"instance setup failed: {errors}")
        for inst in instances:
            print(f"instance {inst.idx}: armed, mode SHOW, T0={t0}", flush=True)

        # phase 4: ride the flow
        for inst in instances:
            inst.wait_text("Show stage: performing", timeout=60)
            print(f"instance {inst.idx}: performing")
        wait_s = args.duration + 90     # show + takeoff/rtl/land margins
        print(f"waiting up to {wait_s}s for each instance to land...")
        for inst in instances:
            try:
                inst.wait_text("Show stage: landed", timeout=wait_s)
                print(f"instance {inst.idx}: landed")
            except RuntimeError as e:
                print(f"instance {inst.idx}: {e} (check {inst.workdir}/sitl.out)")
    finally:
        for inst in instances:
            inst.stop()
        print("instances stopped; logs in %s" % args.out)


if __name__ == '__main__':
    main()
