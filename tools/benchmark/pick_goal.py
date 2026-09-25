#!/usr/bin/env python3
"""Pick a reachable free goal on a ROS map (pgm + yaml) for the ground benchmark.

BFS from the start cell over free space, then chooses the reachable cell with
the best clearance within the requested distance band.

Usage:
    python3 tools/benchmark/pick_goal.py \
        --map-yaml src/ground/sim_env/maps/test2/test2.yaml \
        --min-dist 3.0 --max-dist 5.0
"""

import argparse
import collections
import math
import re


def load_yaml(path):
    text = open(path).read()
    resolution = float(re.search(r"resolution:\s*([0-9.eE+-]+)", text).group(1))
    origin = re.search(r"origin:\s*\[\s*([0-9.eE+-]+)\s*,\s*([0-9.eE+-]+)", text)
    return resolution, float(origin.group(1)), float(origin.group(2))


def load_pgm(path):
    with open(path, "rb") as handle:
        data = handle.read()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            while data[index:index + 1] != b"\n":
                index += 1
            continue
        start = index
        while not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert magic == b"P5", "only binary PGM (P5) is supported"
    index += 1
    pixels = data[index:index + width * height]
    return width, height, pixels


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--min-dist", type=float, default=3.0)
    parser.add_argument("--max-dist", type=float, default=5.0)
    args = parser.parse_args()

    resolution, origin_x, origin_y = load_yaml(args.map_yaml)
    pgm_path = re.sub(r"\.ya?ml$", ".pgm", args.map_yaml)
    width, height, pixels = load_pgm(pgm_path)

    def cell(x, y):
        return int((x - origin_x) / resolution), int((y - origin_y) / resolution)

    def world(c, r):
        return origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution

    def free(c, r):
        if c < 0 or r < 0 or c >= width or r >= height:
            return False
        value = pixels[(height - 1 - r) * width + c]  # pgm rows top-down
        return value >= 250

    start = cell(args.start_x, args.start_y)
    if not free(*start):
        print("start cell is not free:", start)
        return 1

    seen = {start}
    queue = collections.deque([start])
    best = None
    best_score = -1.0
    while queue:
        c, r = queue.popleft()
        x, y = world(c, r)
        distance = math.hypot(x - args.start_x, y - args.start_y)
        if args.min_dist <= distance <= args.max_dist:
            clearance = 0
            for dc in range(-3, 4):
                for dr in range(-3, 4):
                    if free(c + dc, r + dr):
                        clearance += 1
            score = clearance - abs(distance - (args.min_dist + args.max_dist) / 2.0)
            if score > best_score:
                best_score = score
                best = (x, y, distance, clearance)
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (c + dc, r + dr)
            if nxt not in seen and free(*nxt):
                seen.add(nxt)
                queue.append(nxt)

    if best is None:
        print("no reachable goal found in the distance band")
        return 1
    print("GOAL %.2f %.2f (distance %.2f m, clearance %d/49)" % best)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
