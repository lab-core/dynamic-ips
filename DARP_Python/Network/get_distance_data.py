import json
import math
import time
from typing import List, Tuple, Dict, Optional

import pandas as pd
import requests
import geopandas as gpd
import constants as c
import os
from tqdm import tqdm

OSRM_BASE = "https://router.project-osrm.org"
PROFILE = "driving"  # 'driving' | 'walking' | 'cycling'
TIMEOUT = 100  # Increased timeout to 60 seconds
BLOCK_SIZE = 50  # Reduced block size to avoid timeouts (was 90)
MAX_RETRIES = 3  # Number of retry attempts
RETRY_DELAY = 10  # Seconds to wait between retries
REQUEST_DELAY = 1.5


def read_stops_latlon(geojson_path: str) -> Tuple[List[str], List[Tuple[float, float]]]:
    """
    Reads a GeoJSON of points with a 'stop_id' column in WGS84 (EPSG:4326).
    Returns (stop_ids, coords) where coords are (lon, lat).
    """
    gdf = gpd.read_file(geojson_path)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        # If user accidentally passes the projected file, reproject to WGS84.
        gdf = gdf.to_crs(epsg=4326)
    # Ensure Points
    if not all(g.geom_type == "Point" for g in gdf.geometry):
        raise ValueError("All geometries must be Points.")
    stop_ids = gdf["stop_id"].astype(str).tolist()
    coords = [(pt.x, pt.y) for pt in gdf.geometry]  # (lon,lat)
    return stop_ids, coords


def osrm_table_block(
        coords: List[Tuple[float, float]],
        src_idx: List[int],
        dst_idx: List[int],
        base_url: str = OSRM_BASE,
        profile: str = PROFILE,
        timeout: int = TIMEOUT,
        max_retries: int = MAX_RETRIES
) -> Tuple[List[List[float]], List[List[float]]]:
    """
    Query OSRM Table for a subset with retry logic and better error handling.
    """
    # Build the compact coordinate list for this block
    uniq_idx = sorted(set(src_idx) | set(dst_idx))
    local_index = {g: i for i, g in enumerate(uniq_idx)}
    block_coords = [coords[g] for g in uniq_idx]

    # Path segment with coords: "lon,lat;lon,lat;..."
    coord_str = ";".join([f"{lon:.6f},{lat:.6f}" for lon, lat in block_coords])

    # Indices relative to this compact list
    sources_param = ";".join(str(local_index[i]) for i in src_idx)
    destinations_param = ";".join(str(local_index[j]) for j in dst_idx)

    url = f"{base_url}/table/v1/{profile}/{coord_str}"
    params = {
        "sources": sources_param,
        "destinations": destinations_param,
        "annotations": "duration,distance",
    }

    # Retry logic
    for attempt in range(max_retries):
        try:
            r = requests.get(url, params=params, timeout=timeout)
            r.raise_for_status()
            data = r.json()

            if "durations" not in data or "distances" not in data:
                raise ValueError("OSRM response missing 'durations' or 'distances'.")
            time.sleep(REQUEST_DELAY)
            return data["distances"], data["durations"]

        except (requests.exceptions.Timeout, requests.exceptions.RequestException) as e:
            if attempt < max_retries - 1:
                print(f"  Attempt {attempt + 1} failed ({type(e).__name__}), retrying in {RETRY_DELAY}s...")
                time.sleep(RETRY_DELAY)
            else:
                print(
                    f"  All {max_retries} attempts failed for block (sources: {len(src_idx)}, destinations: {len(dst_idx)})")
                raise e


def chunk_indices(n: int, block: int) -> List[List[int]]:
    return [list(range(i, min(i + block, n))) for i in range(0, n, block)]


def estimate_total_time(n_stops: int, block_size: int, avg_request_time: float = 2.0) -> str:
    """
    Estimate total processing time based on number of stops and block size.
    """
    n_src_blocks = math.ceil(n_stops / block_size)
    n_dst_blocks = math.ceil(n_stops / block_size)
    total_requests = n_src_blocks * n_dst_blocks

    estimated_seconds = total_requests * avg_request_time

    if estimated_seconds < 60:
        return f"{estimated_seconds:.0f} seconds"
    elif estimated_seconds < 3600:
        return f"{estimated_seconds / 60:.1f} minutes"
    else:
        return f"{estimated_seconds / 3600:.1f} hours"


def build_full_matrix(
        coords: List[Tuple[float, float]],
        block_size: int = BLOCK_SIZE,
        base_url: str = OSRM_BASE,
        profile: str = PROFILE,
        show_progress: bool = True
) -> Tuple[List[List[float]], List[List[float]]]:
    """
    Builds full NxN distance & duration matrices (directed) by querying OSRM in blocks.
    Returns (dist_matrix, dur_matrix) where both are NxN lists (floats or None).
    """
    n = len(coords)
    print(f"Processing {n} stops with block size {block_size}")

    # Calculate and display estimated time
    estimated_time = estimate_total_time(n, block_size)
    print(f"Estimated processing time: {estimated_time}")

    # Initialize with None to mark unreachable/self prior to filling
    dist = [[None] * n for _ in range(n)]
    dur = [[None] * n for _ in range(n)]

    src_blocks = chunk_indices(n, block_size)
    dst_blocks = chunk_indices(n, block_size)

    total_blocks = len(src_blocks) * len(dst_blocks)
    print(f"Total blocks to process: {total_blocks} ({len(src_blocks)} × {len(dst_blocks)})")

    # Progress tracking
    if show_progress:
        pbar = tqdm(total=total_blocks, desc="Processing blocks", unit="block")

    block_count = 0
    start_time = time.time()

    try:
        for i, sblk in enumerate(src_blocks):
            for j, dblk in enumerate(dst_blocks):
                block_start = time.time()

                try:
                    D_block, T_block = osrm_table_block(
                        coords, sblk, dblk, base_url=base_url, profile=profile
                    )

                    # Fill results. OSRM returns a |sblk| x |dblk| table
                    for i_local, i_global in enumerate(sblk):
                        for j_local, j_global in enumerate(dblk):
                            d_val = D_block[i_local][j_local]
                            t_val = T_block[i_local][j_local]
                            dist[i_global][j_global] = d_val
                            dur[i_global][j_global] = t_val

                    block_count += 1
                    block_time = time.time() - block_start

                    if show_progress:
                        pbar.update(1)
                        pbar.set_postfix({
                            'block_time': f'{block_time:.1f}s',
                            'avg_time': f'{(time.time() - start_time) / block_count:.1f}s'
                        })

                except Exception as e:
                    print(f"\nError processing block ({i + 1},{j + 1}): {e}")
                    # Continue with other blocks, leaving None values for failed block
                    if show_progress:
                        pbar.update(1)
                    continue

    finally:
        if show_progress:
            pbar.close()

    total_time = time.time() - start_time
    print(f"\nCompleted in {total_time:.1f} seconds ({total_time / 60:.1f} minutes)")
    print(f"Successfully processed {block_count}/{total_blocks} blocks")

    return dist, dur


def write_pairs_json(
        out_path: str,
        stop_ids: List[str],
        dist: List[List[float]],
        dur: List[List[float]],
        include_self: bool = False
) -> None:
    """
    Writes an edge-list JSON with one entry per ordered pair.
    Fields: origin_id, destination_id, distance_m, duration_s
    """
    n = len(stop_ids)
    pairs = []
    none_count = 0

    print("Building output pairs...")
    for i in range(n):
        for j in range(n):
            if not include_self and i == j:
                continue

            d_val = dist[i][j]
            t_val = dur[i][j]

            # Track None values (unreachable pairs)
            if d_val is None or t_val is None:
                none_count += 1

            pairs.append({
                "origin_id": stop_ids[i],
                "destination_id": stop_ids[j],
                "distance_m": d_val,
                "duration_s": t_val,
            })

    payload = {
        "profile": PROFILE,
        "count_stops": n,
        "count_pairs": len(pairs),
        "pairs": pairs,
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    if none_count > 0:
        print(f"Warning: {none_count} pairs are unreachable (None values)")


def process_virtual_stops(
        input_geojson_file: str,
        output_json_name: str,
        block_size: int = BLOCK_SIZE,
        profile: str = PROFILE,
        show_progress: bool = True
) -> Tuple[List[str], List[List[float]], List[List[float]]]:
    """
    Process virtual stops from a GeoJSON file and generate distance/duration matrix.

    Args:
        input_geojson_file (str): Path to the input GeoJSON file containing virtual stops
        output_json_name (str): Name/path for the output JSON file
        block_size (int): Size of blocks for OSRM requests (smaller = more reliable)
        profile (str): OSRM profile ('driving', 'walking', 'cycling')
        show_progress (bool): Whether to show progress bars

    Returns:
        tuple: (stop_ids, distance_matrix, duration_matrix) for further processing if needed
    """
    print(f"Starting processing with profile: {profile}")
    print(f"Block size: {block_size}")

    # Load stops from the input GeoJSON file
    input_path = os.path.join(c.STOP_DIR, input_geojson_file)
    stop_ids, coords_lonlat = read_stops_latlon(input_path)

    print(f"Loaded {len(stop_ids)} stops from {input_geojson_file}")

    # Build full directed matrices
    dist_m, dur_s = build_full_matrix(
        coords=coords_lonlat,
        block_size=block_size,
        base_url=OSRM_BASE,
        profile=profile,
        show_progress=show_progress
    )

    output_path = os.path.join(c.STOP_DIR, output_json_name)

    # Save as an edge list JSON (one row per ordered origin→destination)
    write_pairs_json(output_path, stop_ids, dist_m, dur_s, include_self=False)

    print(f"Successfully wrote {output_json_name} with {len(stop_ids)} stops.")

    return stop_ids, dist_m, dur_s


# Additional utility function for resuming interrupted processing
def save_checkpoint(
        checkpoint_path: str,
        stop_ids: List[str],
        dist_matrix: List[List[Optional[float]]],
        dur_matrix: List[List[Optional[float]]],
        completed_blocks: int,
        total_blocks: int
) -> None:
    """Save current progress to a checkpoint file."""
    checkpoint_data = {
        'stop_ids': stop_ids,
        'dist_matrix': dist_matrix,
        'dur_matrix': dur_matrix,
        'completed_blocks': completed_blocks,
        'total_blocks': total_blocks,
        'profile': PROFILE,
        'timestamp': time.time()
    }

    with open(checkpoint_path, 'w') as f:
        json.dump(checkpoint_data, f, indent=2)

    print(f"Checkpoint saved: {completed_blocks}/{total_blocks} blocks completed")


"""
Example Usage:
-------------

    riley_input_path = "riley_virtual_stops_latlon.geojson"
    riley_output_name = "riley_edge_matrix.json"

    input_path = "virtual_stops_latlon.geojson"
    output_name = "edge_matrix.json"

    # Process the stops
    riley_stop_ids, riley_dist_matrix, riley_dur_matrix = process_virtual_stops(riley_input_path, riley_output_name)
    stop_ids, dist_matrix, dur_matrix = process_virtual_stops(input_path, output_name)
    
"""