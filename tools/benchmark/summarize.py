#!/usr/bin/env python3
"""Aggregate benchmark JSON results into mean/std/min/max per metric.

Usage:
    python3 tools/benchmark/summarize.py                       # all run_*.json
    python3 tools/benchmark/summarize.py 'results/ground_*.json'
"""

import glob
import json
import os
import statistics
import sys


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results", "run_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        print("no files matched", pattern)
        return 1

    rows = {}
    for path in files:
        with open(path) as handle:
            data = json.load(handle)
        for key, value in data.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool) and key != "run":
                rows.setdefault(key, []).append(float(value))

    for key in sorted(rows):
        values = rows[key]
        if len(values) > 1:
            print("%-24s n=%d mean=%.4f std=%.4f min=%.4f max=%.4f"
                  % (key, len(values), statistics.mean(values), statistics.pstdev(values),
                     min(values), max(values)))
        else:
            print("%-24s n=1 value=%s" % (key, values[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
