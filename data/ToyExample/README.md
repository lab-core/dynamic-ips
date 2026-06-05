# ToyExample — minimal default instance

A tiny, self-contained Dial-a-Ride instance used as the **default run** of the
C++ solver. Running the executable with **no command-line arguments** loads this
folder and solves it with the B-CG (`DYNAMIC`) workflow:

```bash
cpp/bin/realtime_DARP        # no args -> solves data/ToyExample
```

It is meant for smoke-testing a build and for understanding the input/output
file formats without downloading the full benchmarks. The format mirrors the
real benchmarks (`data/NYC-DARP-Benchmark`, `data/Riley_Benchmark`), just much
smaller.

## The instance

- **Network:** a 4×3 Manhattan-style grid of 12 virtual stops (IDs `0..11`).
  Travel time between two stops is their Manhattan distance × 60 s.
- **Zones:** 4 quadrant zones, each with a representative center stop.
- **Fleet:** 3 vehicles, capacity 4, parked at grid corners.
- **Demand:** 8 requests arriving over the first 3 minutes.

Everything is solver-valid by construction: the duration matrix is a proper
metric, and every stop/zone referenced by a request or vehicle exists.

## Files

| File | Role |
|---|---|
| `edge_time_matrix.txt` | 12×12 travel-time matrix (`nbLocations = 12`) |
| `Zones.txt` | zone id → center stop id |
| `vehicles/vehicles.txt` | 3 vehicles, capacity 4 (fleet size and capacity read from the file) |
| `Instances_toy/toy/INSTANCE_toy.txt` | instance header (counts, sim start) |
| `Instances_toy/toy/TRIP_toy.txt` | the 8 requests |
| `ToyParameters.json` | solver parameters (scenario `toy`) |
| `gen_toy.py` | regenerates all of the above |
| `runs/` | run outputs are written here (git-ignored) |

## Regenerating

```bash
python3 data/ToyExample/gen_toy.py
```

Edit the grid size, fleet, or request list at the top of `gen_toy.py` and rerun
to produce a different toy.

## Running explicitly

The no-arg launch above is equivalent to the following (fleet size and capacity
are left to default from the instance and vehicle file):

```bash
cpp/bin/realtime_DARP \
  --data-dir data/ToyExample --inst-folder Instances_toy --instance-name toy \
  --main-algo B_CG --sol-mode DYNAMIC \
  --paramfile data/ToyExample/ToyParameters --scenario toy \
  --output-dir data/ToyExample/runs
```
