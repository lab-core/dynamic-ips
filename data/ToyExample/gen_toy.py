#!/usr/bin/env python3
"""Generate the ToyExample benchmark files (network, zones, vehicles, instance,
trips) in the same text format the C++ solver reads for the real benchmarks.

The toy network is a 4x3 Manhattan-style grid of 12 virtual stops. Travel time
between two stops is their Manhattan distance times 60 seconds (1 minute per
grid edge), which is a proper metric (symmetric, triangle inequality), so the
column-generation pricing always sees feasible, well-behaved routes.

Run from anywhere; it writes next to this script.
"""

from __future__ import annotations

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# 12 stops on a 4 (cols) x 3 (rows) grid, id = row * 4 + col.
COLS, ROWS = 4, 3
N = COLS * ROWS
UNIT_SECONDS = 60


def coord(loc: int) -> tuple[int, int]:
    return loc % COLS, loc // COLS


def travel_time(i: int, j: int) -> int:
    (xi, yi), (xj, yj) = coord(i), coord(j)
    return (abs(xi - xj) + abs(yi - yj)) * UNIT_SECONDS


# Four quadrant zones; value is a representative center stop used for rebalancing.
ZONE_CENTER = {1: 5, 2: 6, 3: 8, 4: 11}
LOC_ZONE = {
    0: 1, 1: 1, 4: 1, 5: 1,
    2: 2, 3: 2, 6: 2, 7: 2,
    8: 3, 9: 3,
    10: 4, 11: 4,
}

# Three vehicles (capacity 4) parked at corners of the grid.
# Columns: vehicle_ID capacity depart_Time end_Time depart_ID sink_ID zone_ID
VEHICLES = [
    (0, 4, 0, 90000, 0, 0, LOC_ZONE[0]),
    (1, 4, 0, 90000, 3, 3, LOC_ZONE[3]),
    (2, 4, 0, 90000, 8, 8, LOC_ZONE[8]),
]

# Eight requests trickling in over the first three minutes.
# Columns: passenger_count pickup_ID dropoff_ID request_time_sec
# (the pickup/dropoff districts are derived from LOC_ZONE when written).
REQUESTS = [
    (1, 0, 11, 0),
    (2, 3, 8, 30),
    (1, 5, 6, 45),
    (1, 9, 2, 60),
    (2, 1, 10, 90),
    (1, 7, 4, 120),
    (1, 2, 9, 150),
    (1, 6, 0, 180),
]

SIMULATION_START = 0.0


def write_durations() -> None:
    lines = [f"nbLocations = {N}", "", "COLUMNS", "", "startID", "EndID",
             "duration", "", "DURATION_INFO"]
    for i in range(N):
        for j in range(N):
            lines.append(f"{i:4d} {j:4d} {travel_time(i, j):4d}")
    _write("edge_time_matrix.txt", lines)


def write_zones() -> None:
    lines = ["COLUMNS", "", "Zone_ID", "Location_ID", "", "ZONE_INFO"]
    for zone_id, center in ZONE_CENTER.items():
        lines.append(f"{zone_id:3d} {center:4d}")
    _write("Zones.txt", lines)


def write_vehicles() -> None:
    lines = ["COLUMNS", "", "vehicle_ID", "capacity", "depart_Time", "end_Time",
             "depart_ID", "sink_ID", "zone_ID", "", "VEHICLES_INFO"]
    for v in VEHICLES:
        lines.append(" {0} {1} {2} {3} {4:4d} {5:4d} {6:3d}".format(*v))
    # The fleet size and capacity are read from the file contents, not the name.
    _write(os.path.join("vehicles", "vehicles.txt"), lines)


def write_instance() -> None:
    lines = [
        "INSTANCE = toy", "",
        f"SIMULATION_START = {SIMULATION_START}", "",
        f"NUM_VEHICLES = {len(VEHICLES)}",
        "NUM_ONBOARDS = 0",
        "NUM_RECEIVED = 0",
        f"NUM_REQUESTS = {len(REQUESTS)}",
        f"NUM_LOCATIONS = {N}",
    ]
    _write(os.path.join("Instances_toy", "toy", "INSTANCE_toy.txt"), lines)


def write_trips() -> None:
    lines = ["COLUMNS", "", "passenger_count", "pickup_ID", "dropoff_ID",
             "request_time_sec", "pickup_district", "dropoff_district", "",
             "REQUESTS_INFO"]
    for passengers, pick, drop, t in REQUESTS:
        lines.append("{0}  {1:3d} {2:4d} {3:5d} {4:3d} {5:3d}".format(
            passengers, pick, drop, t, LOC_ZONE[pick], LOC_ZONE[drop]))
    _write(os.path.join("Instances_toy", "toy", "TRIP_toy.txt"), lines)


def _write(rel_path: str, lines: list[str]) -> None:
    path = os.path.join(HERE, rel_path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"wrote {rel_path}")


def main() -> None:
    write_durations()
    write_zones()
    write_vehicles()
    write_instance()
    write_trips()


if __name__ == "__main__":
    main()
