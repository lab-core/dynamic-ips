#!/usr/bin/env python3
"""End-to-end regression test for the bundled toy example.

Runs the solver with no arguments (which solves data/ToyExample) and checks that
the resulting summary reports the expected, deterministic outcome: 3 vehicles
serving all 8 requests with none rejected.

Usage:
    regression_toy.py <path-to-realtime_DARP>

The working directory must be the repository root so the binary can locate
data/ToyExample. Exits 0 on success, 1 on failure.
"""

from __future__ import annotations

import csv
import shutil
import subprocess
import sys
from pathlib import Path

RUNS_DIR = Path("data/ToyExample/runs")
EXPECTED_VEHICLES = 3
EXPECTED_SERVED = 8
EXPECTED_REJECTED = 0


def _fail(message: str) -> int:
    print(f"[ FAIL ] toy regression: {message}")
    return 1


def main() -> int:
    if len(sys.argv) != 2:
        return _fail("usage: regression_toy.py <path-to-realtime_DARP>")
    binary = sys.argv[1]

    if not Path("data/ToyExample").is_dir():
        return _fail("data/ToyExample not found; run from the repository root")

    # Start from a clean output tree so we pick up this run's summary.
    shutil.rmtree(RUNS_DIR / "Instances_toy", ignore_errors=True)

    result = subprocess.run([binary], capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout[-2000:])
        print(result.stderr[-2000:])
        return _fail(f"solver exited with code {result.returncode}")

    summaries = sorted(RUNS_DIR.glob("Instances_toy/toy/*/summary_*.csv"))
    if not summaries:
        return _fail("no summary_*.csv produced")

    with summaries[-1].open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return _fail(f"empty summary file: {summaries[-1]}")
    row = rows[0]

    vehicles = int(row["#vehicles"])
    served = int(row["#served Req"])
    rejected = int(row["#Rejected Req"])

    ok = True
    if vehicles != EXPECTED_VEHICLES:
        ok = False
        print(f"  #vehicles = {vehicles}, expected {EXPECTED_VEHICLES}")
    if served != EXPECTED_SERVED:
        ok = False
        print(f"  #served = {served}, expected {EXPECTED_SERVED}")
    if rejected != EXPECTED_REJECTED:
        ok = False
        print(f"  #rejected = {rejected}, expected {EXPECTED_REJECTED}")

    # Clean up generated outputs (also git-ignored).
    shutil.rmtree(RUNS_DIR / "Instances_toy", ignore_errors=True)

    if not ok:
        return _fail("toy solution did not match the expected metrics")
    print(
        f"[ PASS ] toy regression: {vehicles} vehicles, "
        f"{served} served, {rejected} rejected"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
