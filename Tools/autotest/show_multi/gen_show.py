#!/usr/bin/env python3
"""Generate a v2 drone-show event-stream file for the multi-vehicle demo.

Writes a ShowFile v2 file (see libraries/AC_ShowManager/ShowFile.h):
magic + 28B header + a time-ordered event stream of position frames
(type 1) and light frames (type 2).  The choreography is one shape
traced at constant altitude around an optional centre offset: hover at
the origin (takeoff), fly out to the shape, trace it, fly back, hover.

Shapes: rect (closed rectangle), circle, figure8, triangle, diamond.

Usage:
  gen_show.py OUT.bin [duration_s] [--drone-id N] [--shape S]
             [--alt-m M] [--center X,Y] [--frame-ms N]
  gen_show.py --check OUT.bin   # validate a previously generated file
"""
import math
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


# size of each shape for a 60s show, in mm; scales with duration.
# Sizes are modest so the fleet can be arranged with tight spacing and
# the fly-out legs stay below ~1.5m/s (kept inside the drift budget).
SHAPE_R = {
    'rect': 3000,        # half width/height of the rectangle
    'circle': 3500,      # radius
    'figure8': 2000,     # radius of each lobe
    'triangle': 4000,    # circumradius
    'diamond': 3500,     # distance of each vertex from the centre
}


def shape_points(shape, scale, center):
    """Return the closed loop of (x, y) mm traced around 'center' by the
    given shape.  The first and last points are the same (loop closure)."""
    cx, cy = center
    if shape == 'rect':
        r = SHAPE_R['rect'] * scale
        pts = [(-r, -r), (r, -r), (r, r), (-r, r)]
    elif shape == 'circle':
        r = SHAPE_R['circle'] * scale
        pts = [(int(r * math.cos(2 * math.pi * i / 36)),
                int(r * math.sin(2 * math.pi * i / 36))) for i in range(36)]
    elif shape == 'figure8':
        # two full tangent circles: left lobe clockwise (centre -r,0),
        # right lobe counter-clockwise (centre r,0), both starting and
        # ending at the tangent point (0,0) - no crossing segments
        r = SHAPE_R['figure8'] * scale
        pts = []
        for i in range(37):          # 36 segments, closes on (0,0)
            th = 2 * math.pi * i / 36
            pts.append((int(-r + r * math.cos(th)),
                        int(r * math.sin(th))))
        for i in range(37):
            th = 2 * math.pi * i / 36
            pts.append((int(r - r * math.cos(th)),
                        int(-r * math.sin(th))))
    elif shape == 'triangle':
        r = SHAPE_R['triangle'] * scale
        pts = []
        for i in range(3):
            th = -math.pi / 2 + 2 * math.pi * i / 3
            pts.append((int(r * math.cos(th)), int(r * math.sin(th))))
    elif shape == 'diamond':
        r = SHAPE_R['diamond'] * scale
        pts = [(0, -r), (r, 0), (0, r), (-r, 0)]
    else:
        raise SystemExit("unknown shape '%s'" % shape)
    # loop closure: trace back to the first point
    pts.append(pts[0])
    return [(x + cx, y + cy) for (x, y) in pts]


def generate(duration_s=60, drone_id=7, frame_ms=1000, shape='rect',
             alt_m=5.0, center=(0, 0)):
    """Return the full v2 show file for one traced shape.  frame_ms is the
    position-frame spacing (1000ms = 1Hz, 100ms = 10Hz)."""
    duration_ms = duration_s * 1000
    alt_mm = int(-alt_m * 1000)     # NED down-positive
    scale = duration_s / 60.0
    pts = shape_points(shape, scale, center)

    # time budget: hover at origin, fly out, trace the loop, fly back,
    # hover again.  The out/back legs share the fly time with the shape
    # loop: with the modest sizes above and 18m max centre offset the
    # legs stay at ~1.2-1.5m/s.
    hover1_end = int(0.05 * duration_ms)
    go_end = int(0.30 * duration_ms)
    loop_end = int(0.70 * duration_ms)
    back_end = int(0.90 * duration_ms)
    n_seg = len(pts) - 1
    seg_ms = (loop_end - go_end) // n_seg

    waypoints = [(0, 0, 0), (hover1_end, 0, 0), (go_end, pts[0][0], pts[0][1])]
    for k in range(1, n_seg + 1):
        waypoints.append((go_end + k * seg_ms, pts[k][0], pts[k][1]))
    waypoints.append((back_end, 0, 0))
    waypoints.append((duration_ms, 0, 0))

    # position frames every frame_ms
    frames = []                      # (t_ms, type, payload...)
    n_kf = duration_ms // frame_ms + 1
    for i in range(n_kf):
        t = i * frame_ms
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
    frame_ms = 1000
    shape = 'rect'
    alt_m = 5.0
    center = (0, 0)
    if '--drone-id' in args:
        drone_id = int(args[args.index('--drone-id') + 1])
    if '--frame-ms' in args:
        frame_ms = int(args[args.index('--frame-ms') + 1])
    if '--shape' in args:
        shape = args[args.index('--shape') + 1]
        if shape not in SHAPE_R:
            raise SystemExit("unknown shape '%s' (want %s)"
                             % (shape, ','.join(sorted(SHAPE_R))))
    if '--alt-m' in args:
        alt_m = float(args[args.index('--alt-m') + 1])
    if '--center' in args:
        cx, cy = args[args.index('--center') + 1].split(',')
        center = (int(cx), int(cy))
    with open(out, 'wb') as f:
        f.write(generate(duration_s=duration_s, drone_id=drone_id,
                         frame_ms=frame_ms, shape=shape, alt_m=alt_m,
                         center=center))
    print("wrote %s (%us show, %ums frames, shape %s, alt %.1fm, "
          "center %s)" % (out, duration_s, frame_ms, shape, alt_m, center))
    check(out)


if __name__ == '__main__':
    main()
