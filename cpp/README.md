# C++ Solver — Real-time Column Generation for Online Dial-a-Ride / Ridesharing

This is the C++ solver for the **dynamic-ips** project. It solves large-scale
dynamic **Dial-a-Ride Problems (DARP)** within a **rolling-horizon** framework.
The solver implements the two main workflows of the project plus supporting
baselines. 

- **B-CG — Batch-based Column Generation.** 
- **A-CG — Anytime Column Generation.** 
- **Greedy / insertion baseline.** 

The selected algorithm and solution mode are selected via parameters /
command-line options (see [Configuration](#configuration)).
The solver supports commercial backends (**Gurobi** and/or **CPLEX**) for
solving the master problem.

> You must have a valid Gurobi or CPLEX license to use the corresponding
> backend. This repository does not include CPLEX/Gurobi binaries. For best computational performance, the **Gurobi** backend is recommended.

For a high-level overview of the project and datasets, see the
[root README](../README.md). The Python pipeline that prepares data and plots
results is documented in the [Python pipeline](../python/README.md).


---

## Repository structure

```
cpp/
├─ include/                   # vendored third-party headers (json.hpp)
├─ src/
│  ├─ data/                   # core objects (Instance, Request, Vehicle, Route, Parameters, Graph, ...)
│  │  ├─ Graph.cpp/.h
│  │  ├─ Greedy.cpp/.h
│  │  ├─ Instance.cpp/.h
│  │  ├─ Label.cpp/.h
│  │  ├─ Parameters.cpp/.h
│  │  ├─ Request.cpp/.h
│  │  ├─ Route.cpp/.h
│  │  ├─ SolutionMetrics.cpp/.h
│  │  └─ Vehicle.cpp/.h
│  ├─ solvers/                # algorithm implementations
│  │  ├─ CG_Algorithm.cpp/.h
│  │  ├─ LabelingSubProblem.cpp/.h
│  │  ├─ MasterAlgorithm.cpp/.h
│  │  ├─ SubproModeler.cpp/.h
│  │  ├─ GreedyModeler.cpp/.h
│  │  ├─ AnytimeSolver.cpp/.h
│  │  ├─ BatchSolver.cpp/.h
│  │  ├─ OfflineSolver.cpp/.h
│  │  └─ ...
│  ├─ CplexSolver/            # CPLEX-based modeling/solving
│  │  ├─ MP_Cplex.cpp/.h
│  │  ├─ RP_Cplex.cpp/.h
│  │  ├─ CP_Cplex.cpp/.h
│  │  ├─ MIPSolver_Cplex.cpp/.h
│  │  └─ ...
│  ├─ GurobiSolver/           # Gurobi-based modeling/solving
│  │  ├─ MP_Gurobi.cpp/.h
│  │  ├─ RP_Gurobi.cpp/.h
│  │  ├─ CPModeler.cpp/.h
│  │  ├─ MIPSolver_Gurobi.cpp/.h
│  │  └─ ...
│  ├─ utilities/              # config parsing, IO tools, helpers
│  │  ├─ ConfigParser.cpp/.h
│  │  ├─ ReadWrite.cpp/.h
│  │  ├─ Tools.cpp/.h
│  │  └─ ...
│  ├─ realTimeMain.cpp        # entry point (main function)
│  └─ CMakeLists.txt
├─ parameters.md              # parameter reference
├─ CMakeLists.txt
├─ FindEigen3.cmake
├─ FindCPLEX.cmake
└─ FindGUROBI.cmake
```

> Benchmark instance sets live at the repository root under `../data/`, and the
> experiment-generation and SLURM scripts under `../computational_scripts/`.

---

## Requirements

### Core (required)
- **C++17** (or newer)
- **GCC / G++** with C++17 support (e.g. GCC 9+ recommended)
- **CMake** (≥ 3.16 recommended)
- **Eigen3** (header-only linear-algebra library)
- **Boost** (development headers + libraries)

#### Installing Eigen3 and Boost

Eigen3 is header-only, so installing it system-wide is enough:

```bash
# macOS (Homebrew)
brew install eigen boost

# Debian / Ubuntu
sudo apt-get install libeigen3-dev libboost-all-dev
```

CMake locates Eigen3 in this order (see `cpp/CMakeLists.txt`):

1. `find_package(Eigen3)` — finds a system-wide install automatically (e.g. the
   Homebrew/apt packages above; on macOS Homebrew this is
   `/usr/local/include/eigen3` or `/opt/homebrew/include/eigen3`).
2. `-DEIGEN_DIR=/path/to/eigen-3.x.x` passed at configure time.
3. The `EIGEN_DIR` environment variable.

If you see `Eigen3 not found` during configuration, install the package above or
point CMake at the headers explicitly:

```bash
cmake -S . -B build -DEIGEN_DIR=/path/to/eigen-3.x.x
# or
export EIGEN_DIR=/path/to/eigen-3.x.x
```

### Solver backend (choose one)
- **Gurobi Optimizer** (license required) — preferred, and the backend used in
  the experiments, or
- **IBM ILOG CPLEX** (license required)

> You are responsible for installing Gurobi/CPLEX and ensuring your license is
> valid. The custom find-modules `FindGUROBI.cmake` and `FindCPLEX.cmake` are
> used to locate them.

---

## Build

### Configure & compile (Release)

Build from the `cpp/` directory (the commands below assume this). You can also
build from the repository root — the top-level `CMakeLists.txt` forwards to
`cpp/` — in which case the executable is still written to `cpp/bin/`.

The solver backend is selected with `-DSOLVER=<GUROBI|CPLEX>` (defaults to
`GUROBI`).

#### Build with Gurobi (preferred)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLVER=GUROBI
cmake --build build -j
```

#### Build with CPLEX
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLVER=CPLEX
cmake --build build -j
```

---

## Run

The executable is driven entirely by **command-line options** (no positional
arguments). Run with `--help` (or `-h`) to see the full usage message:

```bash
bin/realtime_DARP --help
```

### Required arguments

```text
--data-dir <path>           Root directory containing the benchmark data
--vehicle-folder <path>     Path to vehicle folder (relative to --data-dir)
--inst-folder <path>        Path to instance folder (relative to --data-dir)
--num-vehicles <int>        Number of vehicles (must be positive)
--main-algo <int>           Main algorithm (non-negative integer)
--sol-mode <int>            Solution mode (non-negative integer)
--paramfile <string>        Parameter file name (e.g. AnyParameters or BatchParameters)
--scenario <string>         Scenario name (must match an entry in the parameter JSON)
--output-dir <string>       Root directory for output files
--initial-state <int>       Initial state (non-negative integer)
```

The values accepted by `--main-algo` and `--sol-mode` (e.g. which integer
selects B-CG, A-CG, greedy, MIP, or offline) are documented in the
[Parameter reference](parameters.md).

### Example — run a specific instance

```bash
bin/realtime_DARP --data-dir ../data/NYC-DARP-Benchmark --vehicle-folder vehicles_warmStart_11 \
  --inst-folder Instances_4h-11 --instance-name 20150917_11-240m --num-vehicles 1450 \
  --vehicle-capacity 4 --main-algo 2 --sol-mode 1 --paramfile AnyParameters \
  --scenario Basis_warm_keep --save-scratch 0 --initial-state 1
```

> Run `bin/realtime_DARP --help` for the complete and authoritative list of
> options, including any optional flags shown in the example above.

---

## Configuration

Solver behavior is controlled via **JSON parameter files** (located in
`computational_scripts/parameters/`, e.g. `AnyParameters.json` or
`BatchParameters.json`). A scenario is selected on the command line with
`--paramfile <name> --scenario <key>`; each scenario overrides only the keys it
needs, and all other parameters fall back to their code defaults.

- **Full parameter reference** (types, defaults, descriptions, and the mapping
  from parameters to the B-CG / A-CG settings): [Parameter reference](parameters.md).
- To inspect the exact parameter schema in code:
  - `src/data/Parameters.h` and `src/data/Parameters.cpp`
  - `src/utilities/ConfigParser.h` and `src/utilities/ConfigParser.cpp`

---

## Running experiments

Experiment generation and execution (local and SLURM) are managed from
`computational_scripts/`. The Python helper there does not implement the
algorithm; it only expands experiment matrices into command lines that call this
C++ binary.

See the [Reproducibility guide](../computational_scripts/README_REPRODUCIBILITY.md)
for the full workflow: generating command files, running them locally (including
dry runs and command-range filters), and submitting SLURM array jobs.

---

## Datasets

Both benchmarks are built on **NYC TLC** taxi trip records (2015–2016) using a
Manhattan network of virtual stop locations. The travel-time matrix between stop
locations is provided as `edge_time_matrix.txt` (constructed from OpenStreetMap
routing data).

The benchmark folders live at the repository root under `data/` and are tracked
with [Git LFS](https://git-lfs.com); run `git lfs pull` after cloning to fetch
them. Instructions for downloading the data from Zenodo or regenerating
instances are in the [Python pipeline](../python/README.md).

### Riley_Benchmark

> Riley, C., Legrain, A., & Van Hentenryck, P. (2026). Real-time ride-sharing
> operations — NYC TLC sub Dataset 2015–2016 [Data set]. Zenodo.
> https://doi.org/10.5281/zenodo.18745880

| Folder | Description |
|---|---|
| `Instances_2h-7` | 24 instances, 7:00–9:00 AM (2-hour window) |
| `Instances_2h-11` | 24 instances, 11:00 AM–1:00 PM (2-hour window) |
| `Instances_30s` | Reduced instances (30 epochs, 12:00–1:00 PM) for sensitivity analysis |
| `vehicles_uniform` | Initial vehicle positions for 7:00 AM instances (uniform distribution) |
| `vehicles_byDemand` | Warm-start vehicle states for 11:00 AM instances |

### NYC-DARP-Benchmark

> Amiri, E., Legrain, A., & El Hallaoui, I. (2026). Manhattan Dial-a-Ride
> Benchmark Dataset with NYC TLC Taxi Trips, 2015–2016 (v1.0) [Data set]. Zenodo.
> https://doi.org/10.5281/zenodo.20452171

| Folder | Description |
|---|---|
| `Instances_2h-7` | 24 instances, 7:00–9:00 AM (2-hour window) |
| `Instances_2h-11` | 24 instances, 11:00 AM–1:00 PM (2-hour window) |
| `Instances_4h-11` | 24 instances, 11:00 AM–3:00 PM (4-hour window) |
| `Instances_16h-7` | 24 instances, 7:00 AM–11:00 PM (16-hour window) |
| `vehicles_uniform` | Initial vehicle positions (uniform distribution); used for 7:00 AM and 16-hour instances |
| `vehicles_warmStart_11` | Warm-start vehicle states for 11:00 AM instances |

### Shared conventions

**Naming.** Instance folders follow `YYYYMMDD_HH-Xm`, where `HH` is the start
hour and `X` is the duration in minutes (e.g. `20150926_11-240m` = Sept 26 2015,
11:00 AM start, 4-hour window).

**Warm-up.** For instances starting at 11:00 AM, a one-hour dry run
(10:00–11:00 AM) generates a realistic initial vehicle state; the resulting
vehicle positions and onboard passenger status are saved in the corresponding
warm-start vehicle files.

---

## Outputs and metrics

For each run the solver writes the following files to `--output-dir`, named after the instance with a suffix encoding the solution mode and algorithm (e.g. `<instance>_D_RT_CG.*`).

| File | Contents |
|---|---|
| `LogFinalResults_<tag>.txt` | Human-readable log: parameter echo, per-vehicle routes stop-by-stop, and a final totals/averages block. |
| `summary_<tag>.csv` | One row per run. Service-quality metrics (wait time and trip delay per request/passenger, ...), fleet utilization, CG iteration counts, and runtime breakdown by component. |
| `epochRuntime_<tag>.csv` | One row per epoch. Runtime breakdown (MP, SP, greedy, rebalancing), and labeling statistics (labels generated/dominated/eliminated, pruned arcs/paths). |
| `epochResults_<tag>.csv` | One row per CG iteration within each epoch. LP objective, relative improvement, column pool size, and iteration runtimes — the convergence data used to characterize anytime behavior. |
| `Requests_<tag>.csv` | One row per request. Realized pickup/drop-off times, wait time, trip delay, assigned vehicle, vehicle switches across epochs, and dual value range. |
| `Vehicles_<tag>.csv` | One row per vehicle. Requests served and time breakdown across service, driving full, driving empty, and idle. |
| `Routes_<tag>.csv` | One row per stop visit. Final route plan with planned and realized arrival/departure times, vehicle load, and travel time from the previous stop. |
| `Parameters_<tag>.csv` | One row per run. Machine-readable record of every solver parameter used. Useful for grouping and filtering result sets; columns match the [Parameter reference](parameters.md). |

---


---

## License

Released under the [MIT License](../LICENSE); see the
[root README](../README.md#license). CPLEX/Gurobi remain under their respective
licenses and are not included here.
