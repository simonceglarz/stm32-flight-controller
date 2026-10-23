"""Noise of accel-only vs filtered angle from a still capture.
Usage: python3 tools/rms.py docs/data/tilt_XXXX.csv"""
import csv, sys, statistics

SKIP_S = 2.0   # skip the first 2 s: filter is still settling from its start value

rows = []
with open(sys.argv[1]) as f:
    for r in csv.reader(f):
        try:
            rows.append([float(x) for x in r])   # t_ms, az_g, gx_dps, acc_deg, filt_deg
        except ValueError:
            continue                              # skip header or non-numeric lines

t0 = rows[0][0]
rows = [r for r in rows if (r[0] - t0) / 1000.0 >= SKIP_S]

acc  = [r[3] for r in rows]
filt = [r[4] for r in rows]

# pstdev = RMS of (value - mean), i.e. noise around the average angle
acc_rms, filt_rms = statistics.pstdev(acc), statistics.pstdev(filt)
print(f"samples: {len(rows)}  ({(rows[-1][0]-rows[0][0])/1000:.1f} s)")
print(f"accel-only : mean {statistics.mean(acc):7.3f} deg, RMS noise {acc_rms:.3f} deg, p-p {max(acc)-min(acc):.3f} deg")
print(f"filtered   : mean {statistics.mean(filt):7.3f} deg, RMS noise {filt_rms:.3f} deg, p-p {max(filt)-min(filt):.3f} deg")
print(f"reduction  : {acc_rms/filt_rms:.1f}x")
