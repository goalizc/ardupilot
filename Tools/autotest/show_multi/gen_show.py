#!/usr/bin/env python3
"""Generate a v1 drone-show choreography file for the multi-vehicle demo.

Writes a ShowFile v1 file (see libraries/AC_ShowManager/ShowFile.h):
header + 1 segment + keyframes (1s spacing) + light events.  The
default 60s choreography is a rectangle out-and-back at constant
altitude: hover, fly 20m east, 20m north, back to the origin, hover.

Usage:
  gen_show.py OUT.bin [duration_s] [--drone-id N]
  gen_show.py --check OUT.bin   # validate a previously generated file
"""
import struct
import sys

MAGIC = b"SHOW"
POLY = 0xEDB88320


def crc32(data, crc=0):
    """CRC-32 matching AP_Math crc_crc32: init 0, no final xor."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (POLY & -(crc & 1))
    return crc


def generate(duration_s=60, drone_id=7):
    """Return the full show file bytes for a rectangle out-and-back flight."""
    duration_ms = duration_s * 1000
    alt_mm = -5000          # 5m, NED down-positive
    # displacement scales with duration so the leg speed stays gentle
    # (20m at 60s = ~0.9 m/s); hover first so the aircraft is settled at
    # the origin before the trajectory demands any motion
    span = 20000 * duration_s // 60
    # waypoints (t_ms, x_mm=N, y_mm=E) in the show frame
    hover_end = int(0.20 * duration_ms)     # hover at the origin
    leg1_end = int(0.45 * duration_ms)      # then east
    leg2_end = int(0.70 * duration_ms)      # then north
    leg3_end = int(0.90 * duration_ms)      # return to origin
    east_m = span
    north_m = span
    waypoints = [
        (0, 0, 0),
        (hover_end, 0, 0),
        (leg1_end, 0, east_m),
        (leg2_end, north_m, east_m),
        (leg3_end, 0, 0),
        (duration_ms, 0, 0),
    ]

    # 1s-spaced keyframes with the segment velocity filled in
    keyframes = []
    n_kf = duration_s + 1
    for i in range(n_kf):
        t = i * 1000
        # find the leg containing t
        for wi in range(len(waypoints) - 1):
            (t0, x0, y0) = waypoints[wi]
            (t1, x1, y1) = waypoints[wi + 1]
            if t1 > t0 and t0 <= t <= t1:
                frac = (t - t0) / float(t1 - t0)
                x = int(round(x0 + (x1 - x0) * frac))
                y = int(round(y0 + (y1 - y0) * frac))
                vx = int(round((x1 - x0) * 1000.0 / (t1 - t0)))  # mm/s
                vy = int(round((y1 - y0) * 1000.0 / (t1 - t0)))
                break
        else:
            raise SystemExit("internal waypoint search failed")
        keyframes.append((t, x, y, alt_mm, vx, vy, 0, 0))

    # two overall-colour light events: red at the start, green mid-show
    light_events = [(0, 0, 255, 0, 0), (duration_ms // 2, 0, 0, 255, 0)]

    payload = bytearray()
    payload += struct.pack('<BBHIHHB3x', 1, 0, drone_id, duration_ms,
                           len(keyframes), len(light_events), 1)
    payload += b'\x00\x00\x00\x00'          # crc placeholder (bytes 16..20)
    payload += b'demo' + struct.pack('<II', 0, duration_ms)   # 1 segment
    for kf in keyframes:
        payload += struct.pack('<Iiiihhhh', *kf)
    for le in light_events:
        payload += struct.pack('<IBBBB', *le)
    crc = crc32(payload[0:16])
    crc = crc32(payload[20:], crc)
    payload[16:20] = struct.pack('<I', crc)
    return MAGIC + bytes(payload)


def check(path):
    """Validate a generated file: magic/version/crc/counts/time monotonic."""
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(MAGIC):
        raise SystemExit("bad magic")
    (version, flags, drone_id, duration_ms, kf_count, light_count,
     seg_count) = struct.unpack_from('<BBHIHHB', data, 4)
    if version != 1:
        raise SystemExit("bad version %u" % version)
    stored = struct.unpack_from('<I', data, 20)[0]
    calc = crc32(data[4:20])
    calc = crc32(data[24:], calc)
    if stored != calc:
        raise SystemExit("crc mismatch: stored=%08x calc=%08x" % (stored, calc))
    pos = 24 + seg_count * 12
    last_t = -1
    for i in range(kf_count):
        (t,) = struct.unpack_from('<I', data, pos)
        if t < last_t:
            raise SystemExit("keyframe time not monotonic at %d" % i)
        last_t = t
        pos += 24
    for i in range(light_count):
        (t,) = struct.unpack_from('<I', data, pos)
        pos += 8
    print("OK: version=%u drone_id=%u duration=%ums keyframes=%u lights=%u "
          "segments=%u total=%u bytes" %
          (version, drone_id, duration_ms, kf_count, light_count, seg_count, len(data)))


def main():
    args = sys.argv[1:]
    if len(args) >= 2 and args[0] == '--check':
        check(args[1])
        return
    out = args[0]
    duration_s = int(args[1]) if len(args) > 1 else 60
    drone_id = 7
    if '--drone-id' in args:
        drone_id = int(args[args.index('--drone-id') + 1])
    with open(out, 'wb') as f:
        f.write(generate(duration_s=duration_s, drone_id=drone_id))
    print("wrote %s (%us show)" % (out, duration_s))
    check(out)


if __name__ == '__main__':
    main()
