#!/usr/bin/env python3
"""
plot_pid.py  –  Quick visualizer for steering‑PID logs.

Usage
-----
# Show newest log
$ python3 plot_pid.py

# Show a specific log
$ python3 plot_pid.py pid_2025-05-09_13-42-11.csv
"""

import argparse
import pathlib
import sys
import pandas as pd
import matplotlib.pyplot as plt

# Folder that contains this script and the CSV logs
CSV_DIR = pathlib.Path(__file__).resolve().parent

def newest_csv():
    csv_files = sorted(CSV_DIR.glob("pid_*.csv"))
    return csv_files[-1] if csv_files else None

def main():
    parser = argparse.ArgumentParser(
        description="Plot goal vs. current steering angle from a PID log."
    )
    parser.add_argument(
        "csv",
        nargs="?",
        help="CSV file to open (defaults to most recent pid_*.csv in this folder)",
    )
    args = parser.parse_args()

    csv_path = pathlib.Path(args.csv) if args.csv else newest_csv()
    if not csv_path or not csv_path.exists():
        sys.exit("CSV file not found.")

    df = pd.read_csv(csv_path)
    time_s = df["time_s"] - df["time_s"].iloc[0]  # zero‑based time
    goal = df["goal_steer_deg"]
    curr = df["curr_steer_deg"]

    plt.figure()
    plt.plot(time_s, goal, label="Goal steering (deg)")
    plt.plot(time_s, curr, label="Current steering (deg)")
    plt.xlabel("Time (s)")
    plt.ylabel("Steering angle (deg)")
    plt.title(csv_path.name)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()


# # Plot the most recent log
# python3 plot_pid.py

# # Or plot a specific file
# python3 plot_pid.py pid_2025-05-09_13-42-11.csv