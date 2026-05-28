# Ridesharing Instances from NYCTLC (New York City TLC)

This dataset contains instance sets for dynamic dial-a-ride / ridesharing experiments. The instances are built on the dataset of Riley et al. (2019, 2020), who extracted trip records from the **New York City Taxi and Limousine Commission (NYCTLC)** and defined the study area as **Manhattan** discretized into a grid of **200 m × 200 m** cells, yielding **1718 virtual stop locations** used as pickup and drop-off points. The travel-time matrix between these locations is provided in **`edge_time_matrix.txt`** (startID, EndID, duration in seconds); Riley et al. constructed this matrix using **OpenStreetMap**. The instances are used in the accompanying paper on column-generation-based online dispatching.

---

## Contents Overview

| Folder | Description |
|--------|-------------|
| **Instances_2h-7** | **Set 1:** 24 instances, 7:00–9:00 AM, 2 hours per instance |
| **Instances_2h-11** | **Set 2:** 24 instances, 11:00 AM–1:00 PM, 2 hours per instance |
| **Instances_30s** | **Reduced instances:** 30 epochs (12:00–1:00 PM) for sensitivity analysis |
| **vehicles_uniform** | Initial vehicle configurations (evenly distributed) for Set 1 |
| **vehicles_byDemand_w11** | Warm-up vehicle states for Set 2 (after 10:00–11:00 dry run) |

---

## Set 1 (`Instances_2h-7`)

- **24 instances**, one per date: two days per month from **July 2015 to June 2016**.
- **Time window:** 7:00–9:00 AM (2 hours).
- **Naming:** Each instance folder is `YYYYMMDD_07-120m` (e.g. `20150926_07-120m`).

**Files per instance:**

- `INSTANCE_<id>.txt` — Instance metadata (simulation start time, number of vehicles, requests).
- `TRIP_<id>.txt` — Request list: passenger_count, pickup_ID, dropoff_ID, request_time_sec, pickup_district, dropoff_district.

---

## Set 2 (`Instances_2h-11`)

- Based on the same NYCTLC data; **time window 11:00 AM–1:00 PM** (2 hours).
- **Pre-processing:** Requests originating or terminating outside the study area were excluded. Trips with the same origin and destination and with duration less than 30 seconds were also excluded.
- **24 instances** (same date coverage as Set 1).
- **Time window:** 11:00 AM –1:00 PM (2 hours).
- **Warm-up:** To obtain a realistic initial state, a **one-hour dry run** from **10:00 to 11:00 AM** was performed; vehicle locations and onboard passenger status are saved and used as the initial configuration for the 11:00–1:00 PM simulation (reflected in the vehicle/onboard files in `vehicles_byDemand_w11/`).
- **Naming:** Instance folders `YYYYMMDD_11-120m` (e.g. `20150926_11-120m`).

**Files per instance:** Same structure as Set 1: `INSTANCE_<id>.txt`, `TRIP_<id>.txt`.

---

## Reduced Instances (`Instances_30s`)

- **Purpose:** Sensitivity studies on smaller, computationally manageable instances.
- **Construction:** **30 epochs** are randomly sampled from **12:00–1:00 PM**
- **Folder naming:** `YYYYMMDD_11-120m_<sample_id>` (e.g. `20150926_11-120m_1`).

**Files per reduced instance:**

- `INSTANCE_<id>.txt`, `TRIP_<id>.txt` — Same semantics as above.
- `VEHICLES_<id>.txt` — Vehicle initial state for this reduced instance.
- `ONBOARDS_<id>.txt` — Onboard passenger state after warm-up.
- `WaitRequests_<id>.txt` — Requests still waiting at the start of the reduced horizon.

---

## Vehicle Configuration Files

- **`vehicles_uniform/`**  
  Used for **Set 1**. Files `vehicles_<N>_4.txt` give initial vehicle positions and parameters (vehicle_ID, capacity, depart_Time, end_Time, depart_ID, sink_ID, zone_ID).

- **`vehicles_byDemand_w11/`**  
  Used for **Set 2**. Same naming `vehicles_<N>_4.txt` for fleet size **N** (capacity 4). **ONBOARDS_vehicles_<N>_4.txt** provides the state after the 10:00–11:00 warm-up (location and onboard passengers).

---

## Data Source and References

- **Primary data source:** New York City TLC Trip Record Data (NYCTLC).
- **Spatial and methodological basis:** Manhattan 200 m × 200 m grid and instance design follow:
  - Riley, C., Legrain, A., & Van Hentenryck, P. (2019). Column generation for real-time ride-sharing operations. *CPAIOR*. DOI: [10.1007/978-3-030-19212-9_31](https://doi.org/10.1007/978-3-030-19212-9_31).
  - Riley, C., Van Hentenryck, P., & Yuan, E. (2020). Real-time dispatching of large-scale ride-sharing systems. *IJCAI*. DOI: [10.24963/ijcai.2020/603](https://doi.org/10.24963/ijcai.2020/603).


