#!/usr/bin/env python3
"""Analyse multi-vehicle show alignment from per-instance DF logs.

For each instance log (inst*/logs/*.BIN under the run output directory)
extract:
  - the GPS absolute time (GpsEpoch) of the first SHEV stage=PERFORMING
    event: the moment that instance entered the performance;
  - the SHEV stage timeline;
  - err_xy/err_z statistics from the SHOW periodic messages.

Prints a report with the per-instance start times and the spread
(max-min) of the starts - the measured start-time difference across the
formation.

Usage:
  analyze.py <run-output-dir> [--stage N]
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "modules", "mavlink", "pymavlink"))
from pymavlink import DFReader  # noqa: E402

PERFORMING = 4


def load_events(path):
    """Return (start_epoch_us, stages, show_stats) for one log file."""
    reader = DFReader.DFReader_binary(path, zero_time_base=True)
    start_epoch_us = None
    stages = []            # (gps_epoch_us, stage)
    errs = []
    while True:
        m = reader.recv_match(type=['SHEV', 'SHOW'])
        if m is None:
            break
        if m.get_type() == 'SHEV':
            stages.append((m.GpsEpoch, m.Stage))
            if m.Stage == PERFORMING and start_epoch_us is None:
                start_epoch_us = m.GpsEpoch
        else:              # SHOW
            if m.Stage == PERFORMING:
                errs.append((m.ErrXY, m.ErrZ))
    if start_epoch_us is None:
        start_epoch_us = 0
    n = len(errs)
    if n:
        avg_xy = sum(e[0] for e in errs) / n
        peak_xy = max(e[0] for e in errs)
        avg_z = sum(e[1] for e in errs) / n
    else:
        avg_xy = peak_xy = avg_z = 0.0
    return start_epoch_us, stages, (n, avg_xy, peak_xy, avg_z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', help='run output directory (containing instN/)')
    args = ap.parse_args()

    results = []
    for inst_dir in sorted(glob.glob(os.path.join(args.run_dir, "inst*")),
                           key=lambda p: int(''.join(c for c in os.path.basename(p)
                                                     if c.isdigit()))):
        bins = sorted(glob.glob(os.path.join(inst_dir, "logs", "*.BIN")))
        if not bins:
            print(f"{os.path.basename(inst_dir)}: no logs")
            continue
        for b in bins:
            try:
                start_epoch_us, stages, stats = load_events(b)
            except Exception as e:
                print(f"{os.path.basename(inst_dir)}/{os.path.basename(b)}: "
                      f"read error: {e}")
                continue
            if not stages and stats[0] == 0:
                print(f"{os.path.basename(inst_dir)}/{os.path.basename(b)}: "
                      f"no SHOW/SHEV messages")
                continue
            results.append((os.path.basename(inst_dir), os.path.basename(b),
                            start_epoch_us, stages, stats))

    if not results:
        print("no usable logs found")
        return

    print(f"{'instance':<12}{'start(GpsEpoch us)':<22}{'start rel(ms)':<16}"
          f"{'SHOW rows':<10}{'avg errXY':<10}{'peak errXY':<12}{'avg errZ':<10}")
    base = min(r[2] for r in results if r[2] > 0)
    for (name, fname, start, stages, stats) in results:
        if start > 0:
            rel = (start - base) / 1000.0
        else:
            rel = float('nan')
        (n, avg_xy, peak_xy, avg_z) = stats
        print(f"{name:<12}{start:<22}{rel:<16.3f}{n:<10}{avg_xy:<10.3f}"
              f"{peak_xy:<12.3f}{avg_z:<10.3f}")

    starts = [r[2] for r in results if r[2] > 0]
    if len(starts) >= 2:
        spread_ms = (max(starts) - min(starts)) / 1000.0
        print(f"\nstart-time spread (max-min) across {len(starts)} instances: "
              f"{spread_ms:.3f} ms")
    else:
        print("\nstart-time spread: not computable (need >=2 starts)")

    print("\nstage timelines (instance: gps-relative-s [stage]...):")
    t0 = min(stages[0][0] for _, _, _, stages, _ in results if stages)
    for (name, fname, start, stages, stats) in results:
        if not stages:
            continue
        line = " ".join(f"{((g-t0)/1e6):.1f}s[{s}]" for g, s in stages)
        print(f"  {name:<10} {line}")


if __name__ == '__main__':
    main()
