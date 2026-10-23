#!/usr/bin/env python3
"""
Log CSV lines from the flight controller's serial port, save them, then plot.
Usage:  python3 tools/log_and_plot.py 20      (record for 20 seconds)
Close `screen` first: only one program can open the serial port at a time.
"""
import csv
import sys
import time

import matplotlib.pyplot as plt
import serial                                   # from the pyserial package

PORT = "/dev/ttyACM1"                           # ST-LINK's USB serial port
BAUD = 115200                                   # must match USART2 in CubeMX
seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

rows = []
with serial.Serial(PORT, BAUD, timeout=1) as port:
    port.reset_input_buffer()                   # drop what the OS has buffered
    discard_end = time.time() + 1.0             # ...and for 1 s, read and throw away
    while time.time() < discard_end:            # anything older still arriving from
        port.readline()                         # the ST-LINK's own buffer                   # drop half-received old lines
    t_end = time.time() + seconds
    print(f"Recording {seconds:.0f} s — tilt the board now...")
    while time.time() < t_end:
        line = port.readline().decode(errors="ignore").strip()   # one line of text
        parts = line.split(",")
        if len(parts) != 5:                     # skip anything that isn't a data line
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:                      # a corrupted line: skip it
            continue

if not rows:
    sys.exit("No data received. Is screen still open, or the printf not in CSV form?")

# Save the raw data so the plot can be regenerated or analysed later.
out = f"docs/data/tilt_{time.strftime('%Y%m%d_%H%M%S')}.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_ms", "az_g", "gx_dps", "acc_deg", "filt_deg"])
    w.writerows(rows)
print(f"Saved {len(rows)} samples to {out}")

# Plot accelerometer-only angle vs filtered angle on the same time axis.
t0 = rows[0][0]
t = [(r[0] - t0) / 1000.0 for r in rows]        # ms since first sample -> seconds
plt.plot(t, [r[3] for r in rows], label="accelerometer only", alpha=0.6)
plt.plot(t, [r[4] for r in rows], label="complementary filter", linewidth=2)
plt.xlabel("time (s)")
plt.ylabel("roll angle (deg)")
plt.title("LSM6DSOX roll angle: raw accelerometer vs fused estimate")
plt.legend()
plt.grid(True)
plt.savefig(out.replace(".csv", ".png"), dpi=150)
plt.show()
