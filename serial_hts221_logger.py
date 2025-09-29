#!/usr/bin/env python3
"""
serial_hts221_logger.py
Reads JSON lines from STM32 over a serial port, parses temperature/humidity
from an HTS221, writes to CSV, and (optionally) shows a live plot.

Expected input lines (one per line), for example:
  {"temp_c": 27.82, "hum_rh": 44.6}

Usage examples:
  python serial_hts221_logger.py --port COM6 --baud 115200 --csv data.csv
  python serial_hts221_logger.py --port /dev/ttyUSB0 --baud 115200 --csv data.csv --plot

Requirements:
  pip install pyserial matplotlib
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError as e:
    print("Missing dependency: pyserial. Install with: pip install pyserial")
    sys.exit(1)

# Matplotlib is optional (only if --plot is used)
def _maybe_import_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from collections import deque
        return plt, deque
    except Exception as e:
        print("Warning: matplotlib not available; --plot will be ignored.")
        return None, None

def parse_args():
    p = argparse.ArgumentParser(description="Read HTS221 JSON from UART and log to CSV")
    p.add_argument("--port", required=True, help="Serial port (e.g., COM6 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--csv", default="hts221_log.csv", help="Output CSV path (default: hts221_log.csv)")
    p.add_argument("--plot", action="store_true", help="Show live plot (requires matplotlib)")
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds (default: 1.0)")
    return p.parse_args()

def ensure_csv_header(csv_path):
    try:
        with open(csv_path, "r", newline="", encoding="utf-8") as f:
            first = f.readline()
            if first.startswith("timestamp,"):
                return
    except FileNotFoundError:
        pass
    # Write header
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "temp_c", "hum_rh"])

def main():
    args = parse_args()
    ensure_csv_header(args.csv)

    # Optional plotting
    plt = None
    if args.plot:
        plt_mod, deque = _maybe_import_matplotlib()
        if plt_mod is not None:
            plt = plt_mod
            # Keep last N points in memory for plotting
            N = 200
            xs = deque(maxlen=N)  # timestamps (s since start)
            ys_t = deque(maxlen=N)
            ys_h = deque(maxlen=N)

            fig = plt.figure()
            ax = plt.gca()
            line_t, = ax.plot([], [], label="Temp (°C)")
            line_h, = ax.plot([], [], label="Hum (%RH)")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.set_title("HTS221 live")
            ax.legend(loc="best")
            plt.tight_layout()
        else:
            args.plot = False  # disable plotting if not available

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"ERROR opening serial port: {e}")
        sys.exit(2)

    print(f"Listening on {args.port} @ {args.baud} baud... (Ctrl+C to stop)")
    t0 = time.time()

    with open(args.csv, "a", newline="", encoding="utf-8") as fcsv:
        writer = csv.writer(fcsv)
        try:
            while True:
                try:
                    raw = ser.readline()  # reads until \n or timeout
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    # Some firmware prints other debug lines; only parse those starting with {
                    if not line.startswith("{"):
                        # Print and skip
                        print(f"[DBG] {line}")
                        continue

                    data = json.loads(line)
                    # Accept keys with these common names; be tolerant to case/variants
                    temp = data.get("temp_c")
                    hum = data.get("hum_rh") if "hum_rh" in data else data.get("humidity")

                    if temp is None and hum is None:
                        print(f"[WARN] JSON without expected keys: {line}")
                        continue

                    ts = datetime.now().isoformat(timespec="seconds")
                    writer.writerow([ts, temp, hum])
                    fcsv.flush()

                    # Console short print
                    t_str = f"{temp:.2f}°C" if isinstance(temp, (int, float)) else "NA"
                    h_str = f"{hum:.1f}%RH" if isinstance(hum, (int, float)) else "NA"
                    print(f"{ts}  T={t_str}  H={h_str}")

                    # Update plot if enabled
                    if args.plot and plt is not None:
                        tsec = time.time() - t0
                        xs.append(tsec)
                        ys_t.append(temp if isinstance(temp, (int, float)) else float('nan'))
                        ys_h.append(hum if isinstance(hum, (int, float)) else float('nan'))

                        line_t.set_data(xs, ys_t)
                        line_h.set_data(xs, ys_h)
                        ax = plt.gca()
                        ax.relim()
                        ax.autoscale_view(True, True, True)
                        plt.pause(0.001)

                except json.JSONDecodeError:
                    print(f"[WARN] Bad JSON: {line}")
                except KeyboardInterrupt:
                    print("\nInterrupted by user.")
                    break
                except Exception as e:
                    print(f"[ERR] {e}")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
