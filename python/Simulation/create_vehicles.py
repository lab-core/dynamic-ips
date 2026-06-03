from Simulation.vehicle import *
import json
import pandas as pd
import constants as c
import math
import os
import random
import numpy as np
import copy

CAPACITY = 4

def create_vehicles_files_proportional(network, initial_vehicle, parent_dir, start=0, random_state=42):
    """
    Cascade-down generator (capacity=4):
      3000 -> 2900 -> ... -> 2000 -> 1950 -> 1900 -> ... -> 500
    Each smaller dataset is a strict subset of the previous one, with proportional
    zone_ID counts (largest-remainder rounding).
    """

    rng = np.random.default_rng(random_state)
    CAPACITY = 4

    # --- helper: compute exact per-zone targets for a given total using largest-remainder ---
    def proportional_targets(zone_counts: pd.Series, n_target: int) -> dict:
        base_total = int(zone_counts.sum())
        if base_total == 0:
            return {}
        if n_target == base_total:
            return zone_counts.astype(int).to_dict()

        shares = zone_counts * (n_target / base_total)
        floors = np.floor(shares).astype(int)
        need = n_target - int(floors.sum())
        if need > 0:
            bump = (shares - floors).sort_values(ascending=False).index[:need]
            floors.loc[bump] += 1
        elif need < 0:
            drop = (shares - floors).sort_values(ascending=True).index[:(-need)]
            floors.loc[drop] -= 1
        return floors.astype(int).to_dict()

    # --- build the base dataset (3000, cap=4) ---
    vehicle = Vehicle(
        len(network.districts),
        nb_trip_per_district=initial_vehicle.nb_trip_per_district,
        file_name=f"vehicles_3000_{CAPACITY}",
        nb_vehicles=3000,
        capacity=CAPACITY,
    )
    vehicle.calculate_vehicle_per_district()
    vehicle.create_vehicle_data_from_districts(network=network, start_time=start)

    current_df = copy.deepcopy(vehicle.vehicle_data)  # we'll shrink this step by step

    # Save & (optionally) plot the base
    vehicle.plot_map_vehicle_cells(network, parent_folder=parent_dir, folder_name="vehicles_byDemand_plots")
    vehicle.save_vehicle(folder_name="vehicles_byDemand", parent_dir=parent_dir)

    # --- all target sizes in descending order (no duplicate 2000) ---
    sizes = [3000] + list(range(2900, 1999, -100)) + list(range(1950, 499, -50))

    # --- cascade down: remove from the previously created dataset ---
    for n_target in sizes[1:]:
        # current per-zone counts
        zone_counts = current_df["zone_ID"].value_counts().sort_index()
        targets = proportional_targets(zone_counts, n_target)

        # compute per-zone removals: current - target
        to_drop_idx = []
        for zid, curr_k in zone_counts.items():
            rem = curr_k - int(targets.get(zid, 0))
            if rem <= 0:
                continue
            # sample rows to drop from this zone (without replacement)
            group_idx = current_df.index[current_df["zone_ID"] == zid].to_numpy()
            drop_idx = rng.choice(group_idx, size=rem, replace=False)
            to_drop_idx.append(drop_idx)

        if to_drop_idx:
            to_drop_idx = np.concatenate(to_drop_idx)
            current_df = current_df.drop(to_drop_idx)

        # tidy IDs
        current_df = current_df.sort_index().reset_index(drop=True)
        current_df["vehicle_ID"] = np.arange(len(current_df))
        current_df["capacity"] = CAPACITY  # ensure constant

        # save this size
        vehicle.vehicle_data = current_df.copy()
        vehicle.nb_vehicles = n_target
        vehicle.file_name = f"vehicles_{n_target}_{CAPACITY}"
        vehicle.capacity = CAPACITY

        vehicle.save_vehicle(folder_name="vehicles_byDemand", parent_dir=parent_dir)
        vehicle.plot_map_vehicle_cells(network, parent_folder=parent_dir, folder_name="vehicles_byDemand_plots")


def create_vehicles_files_uniform(network, initial_vehicle, parent_dir, start=0):
    """
    Generate vehicle datasets for capacity=4 with fleet sizes:
      3000, 2900, ..., 2000, 1950, 1900, ..., 500

    DISTRIBUTION RULE (uniform; not proportional):
    - Sort districts by demand (nb_trip_per_district) descending.
    - For each district in that order, sort its cells by cell ID ascending.
    - Place at most ONE vehicle per cell in that order.
    - Concatenate all districts; take the first N cells for the desired fleet size.

    Notes
    -----
    - If the requested fleet size exceeds the number of unique cells available,
      the function will reuse from the beginning (rare; guarded) so generation does not fail.
    - Plotting is executed for each generated dataset (same as your current flow).
    """

    # --- Build a deterministic "universe" ordering of cells (one vehicle per cell) ---
    # Pair each district index with its demand
    demands = list(enumerate(initial_vehicle.nb_trip_per_district))  # [(district_idx, demand), ...]
    # Sort districts by demand desc; stable on ties by district index
    demands.sort(key=lambda x: (-x[1], x[0]))

    ordered_rows = []
    vehicle_id = 0

    for district_idx, _demand in demands:
        # Collect and sort cell IDs for this district
        cell_ids = [int(cell[0]) for cell in network.districts[district_idx].cells]
        cell_ids.sort()

        # One vehicle per cell (uniform) for this district
        for source_id in cell_ids:
            zone_id = network.cell_to_district[int(source_id)]
            ordered_rows.append([
                vehicle_id, CAPACITY, start, 90000, source_id, source_id, zone_id
            ])
            vehicle_id += 1

    # Turn the "universe" into a DataFrame of single-vehicle-per-cell rows
    universe_df = pd.DataFrame(
        ordered_rows,
        columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time', 'depart_ID', 'sink_ID', 'zone_ID']
    )

    # Safety: if a target N exceeds available unique cells, we will wrap around
    def take_first_n(df_universe: pd.DataFrame, n: int) -> pd.DataFrame:
        if n <= len(df_universe):
            out = df_universe.iloc[:n].copy()
        else:
            # Rare fallback: reuse from start to reach n rows
            times = n // len(df_universe)
            rem = n % len(df_universe)
            parts = [df_universe] * times + [df_universe.iloc[:rem]]
            out = pd.concat(parts, axis=0).reset_index(drop=True)

        # Reindex vehicle_ID cleanly and ensure capacity = 4
        out = out.reset_index(drop=True)
        out['vehicle_ID'] = out.index
        out['capacity'] = CAPACITY
        return out

    # --- Instantiate a Vehicle shell; we will overwrite its vehicle_data each time ---
    # We still use Vehicle to save/plot using your existing methods.
    vehicle = Vehicle(len(network.districts),
                      nb_trip_per_district=initial_vehicle.nb_trip_per_district,
                      file_name=f"vehicles_3000_{CAPACITY}",
                      nb_vehicles=3000,
                      capacity=CAPACITY)

    # --- Build the target sizes ---
    targets_100 = list(range(3000, 1999, -100))  # 3000, 2900, ..., 2000
    targets_50  = list(range(1950,  499,  -50))  # 1950, 1900, ..., 500
    all_targets = targets_100 + targets_50

    # --- Generate, save, and plot each dataset ---
    for n in all_targets:
        df_n = take_first_n(universe_df, n)

        vehicle.vehicle_data = df_n
        vehicle.nb_vehicles = n
        vehicle.capacity = CAPACITY
        vehicle.file_name = f"vehicles_{n}_{CAPACITY}"

        vehicle.save_vehicle(folder_name="vehicles_uniform", parent_dir=parent_dir)
        vehicle.plot_map_vehicle_cells(network, parent_folder=parent_dir, folder_name="vehicles_uniform_plots")
