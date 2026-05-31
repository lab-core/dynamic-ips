# DARP_IPS — Real-time Column Generation for Online Dial-a-Ride / Ridesharing

This repository contains a C++ implementation for solving large-scale dial-a-ride / ridesharing problems using **Column Generation (CG)** in a **rolling-horizon (epoch-based)** setting. The approach is designed for real-time re-optimization: at each epoch, a static dial-a-ride problem (DARP) is solved, where the master problem is typically a **set partitioning** formulation and the subproblem generates feasible routes via **labeling / SPPRC-style** techniques.

The codebase supports commercial solver backends (**CPLEX** and/or **Gurobi**) for the master problem modeling/solving.

> You must have a valid license for CPLEX/Gurobi to use those backends.

---

## Key Features
- Column Generation framework (Master Problem + Route/Column generation)
- Labeling / SPPRC subproblem for route generation
- Real-time rolling-horizon execution (epoch-by-epoch re-optimization)
- Optional backends for **Gurobi** (`src/GurobiSolver`) and **CPLEX** (`src/CplexSolver`)
- Utilities for configuration parsing, input/output, and metrics

---

## Repository Structure

```
DARP_IPS/
├─ Riley_Benchmark/           # public benchmark (Riley et al.)
├─ NYC-DARP-Benchmark/        # extended benchmark (Amiri et al.) — keep untracked
├─ src/
│  ├─ data/                   # core objects (Instance, Request, Vehicle, Route, Parameters,...)
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
├─ computational_scripts/    # experiment generation and SLURM submission
├─ docs/                     # parameter reference
├─ realTimeMain.cpp          # entry point (main function)
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
- **Gurobi Optimizer** (license required), or
- **IBM ILOG CPLEX** (license required)

> This repository does not include CPLEX/Gurobi binaries. You are responsible for installing them and ensuring your license is valid.

---

## Build

### 1) Clone
```bash
git clone https://github.com/lab-core/dynamic-ips.git
cd dynamic-ips
```

### 2) Configure & compile (Release)
The project contains custom find-modules (`FindCPLEX.cmake`, `FindGUROBI.cmake`).
Below is the recommended pattern; adapt flags to match your CMake options/targets.

#### Build with Gurobi
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_GUROBI=ON -DUSE_CPLEX=OFF
cmake --build build -j
```

#### Build with CPLEX
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_GUROBI=OFF -DUSE_CPLEX=ON
cmake --build build -j
```

---

## Run

The executable is driven entirely by **command-line options** (no positional arguments).  
Run `--help` (or `-h`) to see the full usage message:

```bash
bin/realtime_DARP --help
```

### Required arguments

The program **requires** all of the following flags:

```text
--data-dir <path>           Root directory containing the benchmark data
--vehicle-folder <path>     Path to vehicle folder (relative to --data-dir)
--inst-folder <path>        Path to instance folder (relative to --data-dir)
--num-vehicles <int>        Number of vehicles (must be positive)
--main-algo <int>           Main algorithm (non-negative integer)
--sol-mode <int>            Solution mode (non-negative integer)
--paramfile <string>        Parameter file name (e.g., AnyParameters or BatchParameters)
--scenario <string>         Scenario name (must match an entry in the parameter JSON)
--output-dir <string>       Root directory for output files
--initial-state <int>       Initial state (non-negative integer)
```

### Example — Run a specific instance
```bash
bin/realtime_DARP --data-dir NYC-DARP-Benchmark --vehicle-folder vehicles_warmStart_11 --inst-folder Instances_4h-11 --instance-name 20150917_11-240m --num-vehicles 1450 --vehicle-capacity 4 --main-algo 2 --sol-mode 1 --paramfile AnyParameters --scenario Basis_warm_keep --save-scratch 0 --initial-state 1
```

---

## Running Experiments

Experiment generation and execution (local and SLURM) are managed from `computational_scripts/`.  
See [`computational_scripts/README_REPRODUCIBILITY.md`](computational_scripts/README_REPRODUCIBILITY.md) for the full workflow, including how to generate command files, run them locally, and submit SLURM array jobs.

---

## Configuration

Solver behavior is controlled via JSON parameter files (located in `computational_scripts/parameters/`). 
See [`docs/parameters.md`](docs/parameters.md) for the full parameter reference with types, defaults, and notes.

To understand the exact parameter schema in code, inspect:
- `src/data/Parameters.h` and `src/data/Parameters.cpp`
- `src/utilities/ConfigParser.h` and `src/utilities/ConfigParser.cpp`

---

## Datasets

Both benchmarks are built on **New York City TLC** taxi trip records (2015–2016), using the Manhattan 200 m × 200 m grid with 1718 virtual stop locations. The travel-time matrix between stop locations is provided in `edge_time_matrix.txt` (constructed from OpenStreetMap).

### Riley_Benchmark

> Riley, C., Legrain, A., & Van Hentenryck, P. (2026). Real-time ride-sharing operations - NYC TLC sub Dataset 2015-2016 [Data set]. Zenodo. https://doi.org/10.5281/zenodo.18745880

| Folder | Description |
|---|---|
| `Instances_2h-7` | 24 instances, 7:00–9:00 AM (2-hour window) |
| `Instances_2h-11` | 24 instances, 11:00 AM–1:00 PM (2-hour window) |
| `Instances_30s` | Reduced instances (30 epochs, 12:00–1:00 PM) for sensitivity analysis |
| `vehicles_uniform` | Initial vehicle positions for 7:00 AM instances (uniform distribution) |
| `vehicles_byDemand` | Warm-start vehicle states for 11:00 AM instances |

### NYC-DARP-Benchmark

> Amiri, E., Legrain, A., & El Hallaoui, I. (2026). Manhattan Dial-a-Ride Benchmark Dataset with NYC TLC Taxi Trips, 2015–2016 (v1.0) [Data set]. Zenodo. https://doi.org/10.5281/zenodo.20452171

| Folder | Description |
|---|---|
| `Instances_2h-7` | 24 instances, 7:00–9:00 AM (2-hour window) |
| `Instances_2h-11` | 24 instances, 11:00 AM–1:00 PM (2-hour window) |
| `Instances_4h-11` | 24 instances, 11:00 AM–3:00 PM (4-hour window) |
| `Instances_16h-7` | 24 instances, 7:00 AM–11:00 PM (16-hour window) |
| `vehicles_uniform` | Initial vehicle positions (uniform distribution); used for 7:00 AM and 16-hour instances |
| `vehicles_warmStart_11` | Warm-start vehicle states for 11:00 AM instances |

### Shared conventions

**Naming:** Instance folders follow `YYYYMMDD_HH-Xm`, where `HH` is the start hour and `X` is the duration in minutes (e.g. `20150926_11-240m` = Sept 26 2015, 11:00 AM start, 4-hour window).

**Warm-up:** For instances starting at 11:00 AM, a one-hour dry run (10:00–11:00 AM) is performed to generate a realistic initial vehicle state. The resulting vehicle positions and onboard passenger status are saved in the corresponding warm-start vehicle files.

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
