# DARP_IPS — Real-time Column Generation for Online Dial-a-Ride / Ridesharing

This repository contains a C++ implementation for solving large-scale dial-a-ride / ridesharing problems using **Column Generation (CG)** in a **rolling-horizon (epoch-based)** setting. The approach is designed for real-time re-optimization: at each epoch, a static assignment/routing problem is solved, where the master problem is typically a **set partitioning** formulation and the subproblem generates feasible routes via **labeling / SPPRC-style** techniques.

The codebase supports commercial solver backends (**CPLEX** and/or **Gurobi**) for the master problem modeling/solving.

> You must have a valid license for CPLEX/Gurobi to use those backends.

---

## Key Features
- Column Generation framework (Master Problem + Route/Column generation)
- Labeling / SPPRC subproblem for route generation
- Real-time rolling-horizon execution (epoch-by-epoch re-optimization)
- Optional backends for **CPLEX** (`src/CplexSolver`) and **Gurobi** (`src/GurobiSolver`)
- Utilities for configuration parsing, input/output, and metrics

---

## Repository Structure

```
DARP_IPS/
├─ datasets/                 # example datasets (if included)
├─ my_datasets/              # local datasets (recommended to keep untracked)
├─ include/                  # shared headers (if applicable)
├─ src/
│  ├─ data/                  # core objects (Graph, Instance, Request, Vehicle, Route, Label, Parameters, metrics...)
│  │  ├─ Graph.cpp/.h
│  │  ├─ Greedy.cpp/.h
│  │  ├─ Instance.cpp/.h
│  │  ├─ Label.cpp/.h
│  │  ├─ Parameters.cpp/.h
│  │  ├─ Request.cpp/.h
│  │  ├─ Route.cpp/.h
│  │  ├─ SolutionMetrics.cpp/.h
│  │  └─ Vehicle.cpp/.h
│  ├─ solvers/               # algorithm implementations
│  │  ├─ CG_Algorithm.cpp/.h
│  │  ├─ LabelingSubProblem.cpp/.h
│  │  ├─ MasterAlgorithm.cpp
│  │  ├─ SubproModeler.cpp/.h
│  │  ├─ GreedyModeler.cpp/.h
│  │  ├─ AnytimeSolver.cpp/.h
│  │  ├─ BatchSolver.cpp/.h
│  │  ├─ OfflineSolver.cpp/.h
│  │  └─ ...
│  ├─ CplexSolver/           # CPLEX-based modeling/solving
│  │  ├─ CPLEXSolver.cpp/.h
│  │  ├─ MIPMasterProblem.cpp/.h
│  │  ├─ CPLEXSubProblem.cpp/.h
│  │  ├─ ReducedProblem.cpp/.h
│  │  └─ ...
│  ├─ GurobiSolver/          # Gurobi-based modeling/solving
│  │  ├─ MP_Gurobi.cpp/.h
│  │  ├─ RP_Gurobi.cpp/.h
│  │  ├─ CP_Gurobi.cpp/.h
│  │  ├─ CPModeler.cpp/.h
│  │  └─ ...
│  └─ utilities/             # config parsing, IO tools, helpers
│     ├─ ConfigParser.cpp/.h
│     ├─ ReadWrite.cpp/.h
│     ├─ Tools.cpp/.h
│     └─ ...
├─ realTimeMain.cpp          # entry point (real-time / rolling horizon)
├─ AnyParameters.json        # example config (real-time)
├─ BatchParameters.json      # example config (batch/offline experiments)
├─ CMakeLists.txt
├─ FindCPLEX.cmake
└─ FindGUROBI.cmake
```

---

## Requirements

### Core (required)
- **C++17** (or newer)
- **GCC / G++** with C++17 support (e.g., GCC 9+ recommended)
- **CMake** (≥ 3.16 recommended)
- **Eigen3** (found via `FindEigen3.cmake` or system installation)
- **Boost** (development headers + libraries)

### Optional solver backends (choose one)
- **IBM ILOG CPLEX** (license required), or
- **Gurobi Optimizer** (license required)

> This repository does not include CPLEX/Gurobi binaries. You are responsible for installing them and ensuring your license is valid.
---

## Build

### 1) Clone
```bash
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>
```

### 2) Configure & compile (Release)
Your project contains custom find-modules (`FindCPLEX.cmake`, `FindGUROBI.cmake`).
Below are common patterns; adapt flags to match your CMake options/targets.

#### Option A — Build with Gurobi
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_GUROBI=ON -DUSE_CPLEX=OFF
cmake --build build -j
```

#### Option B — Build with CPLEX
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CPLEX=ON -DUSE_GUROBI=OFF
cmake --build build -j
```

#### Option C — Build without commercial solvers (if supported)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### If CMake cannot find CPLEX/Gurobi
You may need to define environment variables or cache entries that your `Find*.cmake` expects.

Common examples:
```bash
# Gurobi (example)
export GUROBI_HOME=/path/to/gurobi

# CPLEX (example)
export CPLEX_HOME=/path/to/CPLEX_Studio
```

If you still get errors, open `FindGUROBI.cmake` / `FindCPLEX.cmake` and check which variables they look for.

---

## Run

Your executable is driven entirely by **command-line options** (no positional arguments).  
Run `--help` (or `-h`) to see the full usage message:

```bash
./build/bin/<your_executable> --help
```

### Required arguments

The program **requires** all of the following flags:

```text
--vehicle-folder <path>     Path to vehicle folder
--inst-folder <path>        Path to instance folder
--num-vehicles <int>        Number of vehicles (must be positive)
--main-algo <int>           Main algorithm (non-negative integer)
--sol-mode <int>            Solution mode (non-negative integer)
--paramfile <string>        Parameter file name (e.g., AnyParameters or BatchParameters)
--scenario <string>         Scenario name (must match an entry in the parameter JSON)
--save-scratch <int>        Save scratch flag (0/1/2)
--initial-state <int>       Initial state (non-negative integer)
```

### Example — Run a specific instance
```bash
./build/bin/<your_executable> --vehicle-folder ./vehicles --inst-folder ./instances --instance-name test.txt --num-vehicles 4 --main-algo 1 --sol-mode 0 --paramfile AnyParameters --scenario test --save-scratch 0 --initial-state 1
```

---
## Batch Experiments (predefined scenarios + SLURM)

This repo includes:
- `BatchParameters.json`: default parameters + a library of named **scenarios** (each scenario overrides a subset of parameters).
- `run-batch.sh`: a SLURM array script that sweeps across **groups × instances × vehicle counts × scenarios**.

### Scenarios
Scenarios are defined under `scenarios` in `BatchParameters.json` and can be selected via the CLI flag `--scenario <name>`.

The provided scenario families include (examples):
- `nbPick1..nbPick4`: vary `nbPick` (pickup concurrency/limit)
- `initial_*`, `pruning_*`: small variants for initialization/pruning toggles
- `truncate_*`: uses truncation / label limits and pruning flags
- `dropPick_*`: enables/disables drop-before-pick flexibility
- `dynamic_*`: enables/disables dynamic pricing switches
- `Ab_*`: ablation settings with tighter time limits (e.g., `informTimeLimit`, `pickupDeviationWindow`)
- `commit_*`, `no_commit_*`: variants that differ in commitment / lookahead-related limits
- `multiObj_*`: multi-objective weight settings (`Wait_W1`, `Ride_W2`, `Req_W3`)
- `Cust_W3`: includes a non-zero `Req_W3` weight

To add a new scenario, create a new entry in `BatchParameters.json` under `scenarios`.

### SLURM batch script (`run-batch.sh`)
The script is configured as a SLURM array job and runs **one command per array task**. It loads modules (gcc/eigen/boost/gurobi), defines test groups (e.g., `G1`, `G2`, `G3`), builds a job list, and executes the command matching `SLURM_ARRAY_TASK_ID`.

Typical usage:
```bash
# Run all default groups (as configured in run-batch.sh)
sbatch run-batch.sh

# Run only a subset of groups (comma/space separated)
sbatch --export=SELECTED_GROUPS=G1 run-batch.sh
sbatch --export=SELECTED_GROUPS="G1,G3" run-batch.sh
```

The commands generated by `run-batch.sh` follow this pattern:
```bash
bin/realtime_DARP   --vehicle-folder <folder>   --inst-folder <folder>   --instance-name <instance>   --num-vehicles <N>   --main-algo <id>   --sol-mode <id>   --paramfile <BatchParameters.json>   --scenario <scenarioName>   --save-scratch 1   --initial-state <id>
```

> Note: `run-batch.sh` currently sets `BATCH_PARAMFILE="BatchParameters."`. If your binary expects a JSON filename, change this to `BatchParameters.json`.

## Configuration (`AnyParameters.json`, `BatchParameters.json`)
Solver behavior is controlled via JSON files. Typical parameters include:
- input paths / dataset selection
- epoch length and rolling-horizon settings
- fleet size, capacity, time windows, max waiting time, detour constraints
- solver time limits and CG iteration limits
- objective weights (e.g., wait time + trip delay term)
- logging/output settings
- random seed for reproducibility

To understand the exact parameter schema, inspect:
- `src/data/Parameters.h` and `src/data/Parameters.cpp`
- `src/utilities/ConfigParser.h` and `src/utilities/ConfigParser.cpp`

---

## Input Data
Place datasets in:
- `datasets/` for small public examples, and/or
- `my_datasets/` for larger/local data you do not want to commit.

**Important:** Document your dataset format here once finalized.
A good minimum is:
- expected files and their names
- required columns for requests and vehicles
- time units and coordinate system
- example snippet (few rows)

Implementation references:
- `src/data/Instance.*`
- `src/utilities/ReadWrite.*`

---

## Outputs
The code can report:
- per-epoch computation times
- number of served/assigned requests
- waiting time and other service-quality metrics
- route plans and assignments (depending on output settings)

Check:
- `src/data/SolutionMetrics.*`
- `src/utilities/ReadWrite.*`
  for what is written and where.

---

## Reproducibility Notes
- Use `Release` builds for consistent timing.
- Set a fixed random seed in the JSON config.
- When reporting results, include solver version (CPLEX/Gurobi), commit hash, and config file used.

---

## Citation
If you use this code in academic work, please cite the associated paper:

```bibtex
@article{YOURKEY,
  title   = {<Your paper title>},
  author  = {<Your name(s)>},
  journal = {<Venue>},
  year    = {<Year>},
}
```

---

## License
Add a `LICENSE` file (MIT/BSD/Apache-2.0 are common for research code).
This repository does **not** include CPLEX/Gurobi binaries; those remain under their respective licenses.

---
