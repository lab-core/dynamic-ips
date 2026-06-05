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

The solver backend is selected with `-DSOLVER=<GUROBI|CPLEX|NONE>` (defaults to
`GUROBI`). `NONE` builds the solver-free libraries and tests only (no backend,
no license) — see [Tests](#tests).

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

## Tests

The build includes a small test suite (disable with `-DBUILD_TESTING=OFF`). After
building, run it from the build directory:

```bash
ctest --test-dir build --output-on-failure
```

The unit tests need no solver backend, so you can build and run them without a
Gurobi/CPLEX license by configuring with `-DSOLVER=NONE` (this is what CI does):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLVER=NONE
cmake --build build --target unit_tests
ctest --test-dir build -R unit_tests --output-on-failure
```

Two kinds of tests are registered:

- **`unit_tests`** — C++ unit and regression tests for the self-contained
  pieces (command-line parsing, the vehicle/zone file readers). They link only
  the solver-free `utilities` and `data` libraries, so they build and run
  **without a Gurobi/CPLEX license**. Sources live in [`tests/`](tests/).
- **`toy_regression`** — an end-to-end check that the bundled
  [toy example](../data/ToyExample/README.md) still solves correctly (3 vehicles
  serve all 8 requests, none rejected). It runs the compiled solver, so it is
  only registered when a backend is enabled and requires the corresponding
  license. Run just this one with `ctest --test-dir build -R toy_regression`.

The `unit_tests` build and run on every push / pull request via the
[`build-and-test`](../.github/workflows/build-and-test.yml) GitHub Actions
workflow, on Linux, macOS, and Windows (with `SOLVER=NONE`).

---

## Run

### Quick start — the built-in toy example

Running the executable with **no arguments** solves a tiny bundled instance
([`data/ToyExample`](../data/ToyExample/README.md)) with the B-CG workflow. This
is the fastest way to smoke-test a build:

```bash
# from the repository root (or cpp/, or the build/bin directory)
cpp/bin/realtime_DARP
```

It auto-locates `data/ToyExample`, solves 8 requests with 3 vehicles, and writes
outputs to `data/ToyExample/runs/` (git-ignored). For real experiments, pass
options as below.

### Running your own instances

The executable is otherwise driven entirely by **command-line options** (no
positional arguments). Run with `--help` (or `-h`) to see the full usage message:

```bash
bin/realtime_DARP --help
```

### Required arguments

```text
--inst-folder <path>        Path to instance folder (relative to --data-dir)
--main-algo <int|name>      Main algorithm: 0..4 or a name
                            (GREEDY, MIP, B_CG, F_ICG, A_CG)
--sol-mode <int|name>       Solution mode: 0..2 or a name (STATIC, DYNAMIC, ANYTIME)
--paramfile <string>        Parameter file name (e.g. AnyParameters or BatchParameters)
--scenario <string>         Scenario name (must match an entry in the parameter JSON)
```

`--main-algo` and `--sol-mode` accept either the integer code (backward
compatible) or the case-insensitive enum name. The mapping from these settings
to the B-CG / A-CG / greedy / MIP / offline workflows is documented in the
[Parameter reference](parameters.md).

### Optional arguments

```text
--data-dir <path>           Root data directory (default: current directory)
--vehicle-folder <path>     Vehicle folder, relative to --data-dir (default: vehicles)
--num-vehicles <int>        Fleet size; selects vehicles_<N>_4.txt in the folder.
                            Omit to read the fleet from <folder>/vehicles.txt
--vehicle-capacity <int>    Vehicle capacity (default: the per-vehicle value in the file)
--initial-state <int>       Fleet state at the start of the simulation (default: 0):
                              0 = fresh start (vehicles idle at their depots)
                              1 = warm start (precomputed state + onboard passengers)
                              2 = resume from a saved mid-simulation state
--instance-name <string>    Specific instance (default: read instance list from file)
--output-dir <path>         Output root (default: next to the instance data;
                            or set DARP_OUTPUT_DIR)
```

**Fleet size and capacity come from the vehicle file, not its name.** With
`--num-vehicles N`, the file `vehicles_<N>_4.txt` is selected (the benchmark
convention for choosing a fleet size). When `--num-vehicles` is omitted, the
solver reads `<vehicle-folder>/vehicles.txt` and takes the fleet size from its
row count. Likewise, `--vehicle-capacity` overrides the capacity only when given;
otherwise the per-vehicle capacity column of the file is used as-is.

**`--initial-state`** selects how the fleet is initialized:

| Value | Meaning |
|---|---|
| `0` | **Fresh start** — vehicles begin idle at their depots with no passengers. |
| `1` | **Warm start** — load a precomputed initial fleet state and onboard passengers from the general `ONBOARDS_<file>` in the vehicle folder (e.g. the `vehicles_warmStart_11` sets). |
| `2` | **Resume** — continue from a saved mid-simulation state, using the instance-specific `VEHICLES_`/`ONBOARDS_`/`WaitRequests_` files. |

### Example — run a specific instance

```bash
# fleet size from the instance, algo/mode by name
bin/realtime_DARP --data-dir ../data/NYC-DARP-Benchmark --vehicle-folder vehicles_warmStart_11 \
  --inst-folder Instances_4h-11 --instance-name 20150917_11-240m --num-vehicles 1450 \
  --main-algo B_CG --sol-mode DYNAMIC --paramfile ../computational_scripts/parameters/AnyParameters \
  --scenario Basis_warm_keep --initial-state 1
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

| Folder | Description                                                                     |
|---|---------------------------------------------------------------------------------|
| `Instances_2h-7` | 24 instances, 7:00–9:00 AM (2-hour window)                                      |
| `Instances_2h-11` | 24 instances, 11:00 AM–1:00 PM (2-hour window)                                  |
| `Instances_30s` | Reduced instances (30 epochs, 12:00–1:00 PM) for sensitivity analysis           |
| `vehicles_uniform` | Initial vehicle positions for 7:00 AM instances (uniform distribution)          |
| `vehicles_byDemand` | Initial vehicle positions for 7:00 AM instances (random distribution by demand) |
| `vehicles_warmStart_11` | Warm-start vehicle states for 11:00 AM instances                                |

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

For each run the solver writes the following files to `--output-dir`, named after the instance with a suffix encoding the solution mode and algorithm (e.g. `<instance>_D_B_CG.*`).

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
