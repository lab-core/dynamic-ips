# DARP_Python — Data Preparation and Result Analysis

Python pipeline for the **dynamic-ips** project.  It covers two independent
workflows:

1. **Data preparation** — build the network, fetch raw trip records, transform
   them, and generate solver-ready benchmark instances.
2. **Result analysis** — post-process solver output and produce all figures for
   the B-CG and A-CG analyses.

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

## Section 1 — Reproduce figures (reviewer path)

This path assumes benchmark results already exist under `Outputs/`.
Download pre-computed results from Zenodo and place them there, then run:

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

## Section 2 — Full pipeline from raw data

### Step 0 — Download data from Zenodo

**Your own network (NYC-DARP):**
Download from *(Zenodo link — to be added)* and unzip into:
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

---

### Step 1 — Build network (custom network only)

If you downloaded the pre-built network files from Zenodo, skip this step.
To rebuild the custom network from scratch:

```bash
python scripts/01_build_network.py           # all steps
python scripts/01_build_network.py --steps stops     # virtual stops only
python scripts/01_build_network.py --steps matrix    # OSRM matrix only
python scripts/01_build_network.py --steps postprocess  # post-process only
```

> **Note:** Step 3 (matrix) requires an OSRM server.
> The public server at `router.project-osrm.org` can be used for small runs
> but may be slow for a full 1 700-stop matrix.

Riley's network does not require this step — the matrix is provided on Zenodo.

---

### Step 2 — Download raw trip records

```bash
python scripts/02_fetch_trips.py                # all configured dates (2015 + 2016)
python scripts/02_fetch_trips.py --year 2015    # 2015 only
python scripts/02_fetch_trips.py --year 2016    # 2016 only
```

Dates are configured in `constants.py` (`DATES_2015`, `DATES_2016`).
Records are saved to `Data/days/`.

> Set `SODA_APP_TOKEN` in your environment to raise the Socrata API rate limit.

---

### Step 3 — Transform trip records

Assigns virtual stop IDs to each trip using a nearest-stop lookup.

```bash
python scripts/03_transform_trips.py --network own    # custom network
python scripts/03_transform_trips.py --network riley  # Riley's network
```

Output goes to `Data/transform_days/`.

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

See `DARP_IPS/README.md` for build instructions and how to run the C++ solver
on the generated instances.

---

### Step 6 — Plot results

After the solver has finished, merge epoch results and generate figures:

```bash
python scripts/plot_BCG.py --gather-data --folders all
python scripts/plot_ACG.py --gather-data --folders all
```

---

## Section 3 — Data sources

### NYC-DARP (custom network)

| File | Description |
|------|-------------|
| `virtual_stops_latlon.geojson` | Custom virtual stop locations (WGS84) |
| `edge_matrix.json` | Raw OSRM pairwise travel-time / distance matrix |
| `edge_time_matrix.txt` | Solver-ready duration matrix (post-processed) |
| `taxi_zones.geojson` | NYC TLC Manhattan taxi-zone polygons |
| `Data/days/*.json` | Raw NYC TLC taxi records (2015–2016) |

### Riley et al. network

| File | Description | Source |
|------|-------------|--------|
| `riley_virtual_stops_latlon.geojson` | 1 718-cell Manhattan grid stops | Zenodo |
| `edge_time_matrix.txt` | OSRM travel-time matrix (OSRM + OSM) | Zenodo |

Both networks share the same NYC TLC taxi-zone district definitions
and the same raw day record files.

---

## Configuration

All paths, dates, and scenario mappings are centralised in `constants.py`.
Adjust `DATA_DIR`, `OUTPUT_DIR`, `DATES_2015`, and `DATES_2016` there
before running the pipeline.
