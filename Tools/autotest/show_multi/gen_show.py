#!/usr/bin/env python3
"""Generate a v2 drone-show event-stream file for the multi-vehicle demo.

Writes a ShowFile v2 file (see libraries/AC_ShowManager/ShowFile.h):
magic + 28B header + a time-ordered event stream of position frames
(type 1) and light frames (type 2).  The default 60s choreography is a
rectangle out-and-back at constant altitude: hover, fly east, fly
north, back to the origin, hover.

Usage:
  gen_show.py OUT.bin [duration_s] [--drone-id N]
  gen_show.py --check OUT.bin   # validate a previously generated file
"""
import struct
import sys

MAGIC = b"SHOW"
POLY = 0xEDB88320
HEADER_SIZE = 28
CRC_OFFSET = 24          # relative to the byte after the magic
EVENT_POSITION = 1
EVENT_LIGHT = 2


def crc32_stream(data, crc=0):
    """CRC over file bytes, skipping the magic [0,4) and the stored crc
    field [4+CRC_OFFSET, 4+CRC_OFFSET+4) - mirrors the firmware."""
    for i, byte in enumerate(data):
        off = i
        if off < 4 or (4 + CRC_OFFSET <= off < 4 + CRC_OFFSET + 4):
            continue
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (POLY & -(crc & 1))
    return crc


def generate(duration_s=60, drone_id=7):
    """Return the full v2 show file for a rectangle out-and-back flight."""
    duration_ms = duration_s * 1000
    alt_mm = -5000          # 5m, NED down-positive
    span = 20000 * duration_s // 60          # displacement scales with duration
    hover_end = int(0.20 * duration_ms)
    leg1_end = int(0.45 * duration_ms)
    leg2_end = int(0.70 * duration_ms)
    leg3_end = int(0.90 * duration_ms)
    waypoints = [
        (0, 0, 0),
        (hover_end, 0, 0),
        (leg1_end, 0, span),
        (leg2_end, span, span),
        (leg3_end, 0, 0),
        (duration_ms, 0, 0),
    ]

    # 1s-spaced position frames
    frames = []                      # (t_ms, type, payload...)
    n_kf = duration_s + 1
    for i in range(n_kf):
        t = i * 1000
        for wi in range(len(waypoints) - 1):
            (t0, x0, y0) = waypoints[wi]
            (t1, x1, y1) = waypoints[wi + 1]
            if t1 > t0 and t0 <= t <= t1:
                frac = (t - t0) / float(t1 - t0)
                x = int(round(x0 + (x1 - x0) * frac))
                y = int(round(y0 + (y1 - y0) * frac))
                vx = int(round((x1 - x0) * 1000.0 / (t1 - t0)))
                vy = int(round((y1 - y0) * 1000.0 / (t1 - t0)))
                break
        else:
            raise SystemExit("internal waypoint search failed")
        frames.append((t, EVENT_POSITION, x, y, alt_mm, vx, vy, 0, 0))

    # two overall-colour light events
    frames.append((0, EVENT_LIGHT, 0, 255, 0, 0))
    frames.append((duration_ms // 2, EVENT_LIGHT, 0, 0, 255, 0))
    frames.sort(key=lambda f: (f[0], f[1]))   # position before light at same t

    n_pos = sum(1 for f in frames if f[1] == EVENT_POSITION)
    n_light = sum(1 for f in frames if f[1] == EVENT_LIGHT)

    b = bytearray()
    b += MAGIC
    b += struct.pack('<BBHIIIIB3x', 2, 0, drone_id, duration_ms,
                     n_pos + n_light, n_pos, n_light, 0)
    b += b'\x00\x00\x00\x00'                  # crc placeholder [28,32)
    for f in frames:
        t, typ = f[0], f[1]
        if typ == EVENT_POSITION:
            _, _, x, y, z, vx, vy, vz, yaw = f
            b += struct.pack('<BIiiihhhh', typ, t, x, y, z, vx, vy, vz, yaw)
        else:
            _, _, index, r, g, bl = f
            b += struct.pack('<BIBBBB', typ, t, index, r, g, bl)
    crc = crc32_stream(bytes(b))
    b[4 + CRC_OFFSET:4 + CRC_OFFSET + 4] = struct.pack('<I', crc)
    return bytes(b)


def check(path):
    """Validate: magic/version/crc/counts/t-time monotonic/type known."""
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(MAGIC):
        raise SystemExit("bad magic")
    (version, _flags, drone_id, duration_ms, n_events, n_pos, n_light,
     _n_seg) = struct.unpack_from('<BBHIIIIB', data, 4)
    if version != 2:
        raise SystemExit("bad version %u" % version)
    if n_events != n_pos + n_light:
        raise SystemExit("event count mismatch")
    stored = struct.unpack_from('<I', data, 28)[0]
    if stored != crc32_stream(data):
        raise SystemExit("crc mismatch: stored=%08x" % stored)
    pos = 32
    last_t = -1
    for _ in range(n_events):
        (typ, t) = struct.unpack_from('<BI', data, pos)
        if typ not in (EVENT_POSITION, EVENT_LIGHT):
            raise SystemExit("unknown event type %u" % typ)
        if t < last_t:
            raise SystemExit("event time not monotonic")
        last_t = t
        pos += 5
        if typ == EVENT_POSITION:
            pos += 20
        else:
            pos += 4
    if pos != len(data):
        raise SystemExit("trailing bytes: %u" % (len(data) - pos))
    print("OK: v2 drone_id=%u duration=%ums events=%u (pos=%u light=%u) "
          "total=%u bytes" % (drone_id, duration_ms, n_events, n_pos, n_light, len(data)))


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
