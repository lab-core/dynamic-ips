# DARP_Python — Data Preparation and Result Analysis

Python pipeline for the **dynamic-ips** project. It covers two independent
workflows:

1. **Data preparation** — build the network, fetch raw trip records, transform
   them, and generate solver-ready benchmark instances.
2. **Result analysis** — post-process solver output and produce all figures for
   the **B-CG** (Batch-based Column Generation) and **A-CG** (Anytime
   Column Generation) analyses.

This pipeline does **not** implement the optimization algorithm; the solver
itself is the C++ code in [`../DARP_IPS`](../DARP_IPS/README.md). For an
overview of the whole project, see the [root README](../README.md).

---

## Prerequisites

```bash
pip install -r requirements.txt
```

Python 3.9 or later is required.

---

## Directory layout

```
DARP_Python/
├── scripts/                    CLI entry points (start here)
│   ├── 01_build_network.py     Build custom network (own data only)
│   ├── 02_fetch_trips.py       Download NYC taxi day records
│   ├── 03_transform_trips.py   Assign virtual stop IDs to records
│   ├── 04_create_instances.py  Generate benchmark instance files
│   ├── plot_BCG.py             Reproduce B-CG figures
│   └── plot_ACG.py             Reproduce A-CG figures
├── Network/                    Network construction modules
├── Simulation/                 Trip preprocessing and instance generation
├── ProcessResults/             Result merging and post-processing
├── Visualization/              All plotting functions
├── main_network.py             Network build entry point (legacy)
├── main_instance.py            Instance creation entry point (legacy)
├── main_transform_days.py      Data transformation entry point (legacy)
├── batch_CG_postprocess.py     B-CG plot functions
├── any_CG_postprocess.py       A-CG plot functions
└── constants.py                Paths, dates, and configuration constants
```

---

## Section 1 — Reproduce figures

This path assumes benchmark results already exist under `Outputs/`. The following options are available:

**B-CG figures:**
```bash
python scripts/plot_BCG.py --folders all
```

**A-CG figures:**
```bash
python scripts/plot_ACG.py --folders all
```

Specific figure groups only:
```bash
python scripts/plot_BCG.py --folders ablation multiObj
python scripts/plot_ACG.py --folders reOptimize rebalance_anytime
```

Run `python scripts/plot_BCG.py --help` or `python scripts/plot_ACG.py --help`
for the full list of available folder keys.

---

## Section 2 — Building benchmark instances

There are two ways to obtain solver-ready instances, depending on your goal:

- **Reproduce results / compare against these benchmarks (recommended).** The
  prepared networks and processed trip data for both the **NYC-DARP** (this
  work) and **Riley** networks are already published on Zenodo. Download them
  (Step 0), then go straight to **[Step 4](#step-4--create-benchmark-instances)**.
  Steps 1–3 are **not** needed.
- **Build your own network or dataset.** Only if you want to generate a *new*
  network or use *different* dates / settings, run the optional
  **[Steps 1–3](#steps-13-optional--build-your-own-network-and-data)** first,
  then continue with Step 4.

### Step 0 — Download prepared data from Zenodo

**NYC-DARP network (this work):**
Download from Zenodo ([doi:10.5281/zenodo.20452171](https://doi.org/10.5281/zenodo.20452171))
and unzip into:
```
DARP_Python/
└── Data/
    ├── stops/                  virtual_stops_latlon.geojson, edge_matrix.json, ...
    └── taxi_zones/             NYC TLC taxi zone shapefile
```

**Riley et al. network:**
Download from [doi:10.5281/zenodo.18745880](https://doi.org/10.5281/zenodo.18745880)
and unzip into:
```
DARP_Python/
└── Data/
    └── manhattan-network/      riley_virtual_stops_latlon.geojson,
                                edge_time_matrix.txt, ...
```

Once the data is in place, skip to [Step 4](#step-4--create-benchmark-instances).

---

### Steps 1–3 (optional) — Build your own network and data

These steps are only required when generating a network or dataset from scratch
(e.g. a different region, dates, or stop settings). **If you downloaded the
prepared data in Step 0, skip them.** Dates and paths are configured in
`constants.py` (`DATES_2015`, `DATES_2016`, `DATA_DIR`, ...).

```bash
# Step 1 — Build the custom network (own data only).
python scripts/01_build_network.py            # all steps

# Step 2 — Download raw NYC TLC trip records → Data/days/
python scripts/02_fetch_trips.py              # all configured dates

# Step 3 — Assign virtual stop IDs to each trip → Data/transform_days/
python scripts/03_transform_trips.py --network own 
```

---

### Step 4 — Create benchmark instances

Generates solver-ready TRIP, REQUESTS, INSTANCE, and vehicle fleet files.

```bash
# Custom network, 07:00–09:00 window (default):
python scripts/04_create_instances.py --network own

# Riley's network, 11:00–15:00 window:
python scripts/04_create_instances.py --network riley \
    --start-hour 11 --end-hour 15 --folder Instances_4h-11

# Skip vehicle file generation:
python scripts/04_create_instances.py --network own --no-vehicles
```

---

### Step 5 — Run the solver

Build and run the C++ solver on the generated instances. See
[`../DARP_IPS/README.md`](../DARP_IPS/README.md) for build instructions and how
to run, and
[`../DARP_IPS/computational_scripts/README_REPRODUCIBILITY.md`](../DARP_IPS/computational_scripts/README_REPRODUCIBILITY.md)
for generating and running experiment commands at scale.

---

### Step 6 — Plot results

After the solver has finished, merge epoch results and generate figures:

```bash
python scripts/plot_BCG.py --gather-data --folders all
python scripts/plot_ACG.py --gather-data --folders all
```

---

## Configuration

All paths, dates, and scenario mappings are centralized in `constants.py`.
Adjust `DATA_DIR`, `OUTPUT_DIR`, `DATES_2015`, and `DATES_2016` there before
running the pipeline.

---

## Expected inputs and outputs

- **Inputs:** network files and raw trip records under `Data/` (from Zenodo, or
  rebuilt via the optional Steps 1–3); solver output under `Outputs/` for the
  plotting path.
- **Outputs:** solver-ready benchmark instances (TRIP, REQUESTS, INSTANCE, and
  vehicle fleet files) from Step 4, and the B-CG / A-CG figures from the
  plotting scripts. The `--gather-data` flag first merges per-epoch solver
  results before plotting.

---

## Related documentation

- [Root README](../README.md) — project overview and datasets.
- [`../DARP_IPS/README.md`](../DARP_IPS/README.md) — build and run the C++ solver.
- [`../DARP_IPS/computational_scripts/README_REPRODUCIBILITY.md`](../DARP_IPS/computational_scripts/README_REPRODUCIBILITY.md) — generate and run experiment commands.
- [`../DARP_IPS/docs/parameters.md`](../DARP_IPS/docs/parameters.md) — solver parameter reference.