"""
Build the custom NYC virtual-stop network and travel-time matrix.

This script is only required for the **custom network** (NYC-DARP dataset).
Riley's network data (stops + travel-time matrix) is available directly from
Zenodo and does not need to be rebuilt.

Pipeline executed by this script:
  1. Download Manhattan taxi-zone shapefile from NYC open data
  2. Extract virtual stop locations from OpenStreetMap (OSMnx)
  3. Query OSRM for pairwise travel-time / distance matrix
  4. Post-process: enforce triangle inequality, export solver-ready TXT

Usage
-----
    python scripts/01_build_network.py                    # run all steps
    python scripts/01_build_network.py --steps stops      # only steps 1-2
    python scripts/01_build_network.py --steps matrix     # only step 3
    python scripts/01_build_network.py --steps postprocess # only step 4

Prerequisites
-------------
  - OSRM server accessible (default: http://router.project-osrm.org)
  - OSMnx, geopandas, requests installed (see requirements.txt)
  - For Riley's network: download data from https://doi.org/10.5281/zenodo.18745880
    and place in Data/manhattan-network/
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Network.create_network import (
    create_taxi_zone_shapefile,
    create_virtual_stops,
    create_riley_virtual_stops_from_real_points,
)
from Network.get_distance_data import process_virtual_stops
from Network.post_process_network import postprocess_json_matrix, convert_json_matrix_to_txt
from Visualization.visualize_network import plot_virtual_stops
import constants as c


def build_stops():
    """Steps 1-2: create taxi-zone shapefile and virtual stop locations."""
    print("[01] Creating taxi-zone shapefile …")
    create_taxi_zone_shapefile()

    print("[01] Creating custom virtual stops from OSM …")
    create_virtual_stops()

    print("[01] Creating Riley virtual stops from real points …")
    create_riley_virtual_stops_from_real_points()

    print("[01] Plotting stop maps …")
    plot_virtual_stops(
        output_filename="virtual_stops_with_zoom.pdf",
        input_stop_file="virtual_stops.geojson",
        zoom_bounds=(585000, 588000, 4513000, 4516000),
    )
    plot_virtual_stops(
        output_filename="riley_virtual_stops_with_zoom.pdf",
        input_stop_file="riley_virtual_stops.geojson",
        zoom_bounds=(585000, 588000, 4513000, 4516000),
        stop_color="maroon",
    )
    print("[01] Stop files written to", c.STOP_DIR)


def build_matrix():
    """Step 3: query OSRM for the full pairwise travel-time matrix."""
    print("[01] Querying OSRM for custom-network matrix …")
    process_virtual_stops("virtual_stops_latlon.geojson", "edge_matrix.json")
    convert_json_matrix_to_txt("edge_matrix.json", "edge_time_matrix")
    print("[01] Raw matrix written to edge_matrix.json / edge_time_matrix.txt")


def build_postprocess():
    """Step 4: enforce triangle inequality and write solver-ready files."""
    print("[01] Post-processing matrix (triangle inequality) …")
    postprocess_json_matrix("edge_matrix.json")
    print("[01] Done — solver-ready files updated.")


def main():
    parser = argparse.ArgumentParser(
        description="Build custom NYC virtual-stop network and travel-time matrix.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--steps",
        nargs="+",
        choices=["stops", "matrix", "postprocess", "all"],
        default=["all"],
        help="Which pipeline steps to run (default: all).",
    )
    args = parser.parse_args()

    run_all = "all" in args.steps
    if run_all or "stops" in args.steps:
        build_stops()
    if run_all or "matrix" in args.steps:
        build_matrix()
    if run_all or "postprocess" in args.steps:
        build_postprocess()

    print("[01] Network build complete.")


if __name__ == "__main__":
    main()
