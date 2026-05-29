"""
Create solver-ready benchmark instances from transformed trip records.

For each transformed day file the script:
  - loads the network (districts + stop locations + travel-time matrix)
  - filters trips to the requested time window
  - splits trips exceeding vehicle capacity into multiple requests
  - removes trips where pickup == drop-off
  - writes TRIP, REQUESTS, and INSTANCE files for the solver
  - generates proportional and uniform vehicle deployment files for all
    fleet sizes (500 – 3 000 vehicles)

The --network flag selects which network files to load:

  own    — custom network files in Data/stops/
  riley  — Riley et al. files in Data/manhattan-network/

Usage
-----
    # Custom network, 7 AM – 9 AM window (default):
    python scripts/04_create_instances.py --network own

    # Riley's network, 11 AM – 3 PM window:
    python scripts/04_create_instances.py --network riley \\
        --start-hour 11 --end-hour 15 --folder Instances_4h-11

    # Suppress vehicle file generation:
    python scripts/04_create_instances.py --network own --no-vehicles
"""

import sys
import os
import argparse
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Network.network import District_Network
from Simulation.dataset import Dataset
from Simulation.vehicle import Vehicle
from Simulation.create_vehicles import (
    create_vehicles_files_proportional,
    create_vehicles_files_uniform,
)
import Simulation.utilities as uf
import constants as c

# Network file names per network variant
_NETWORK_FILES = {
    "own": {
        "geo":          "taxi_zones",
        "stops_geojson": "virtual_stops_latlon",
        "edge_matrix":  "edge_matrix",
        "transform_dir": c.TRANSFORM_DAY_DIR,
    },
    "riley": {
        "geo":          "taxi_zones",
        "stops_geojson": os.path.join(c.RILEY_DIR, "riley_virtual_stops_latlon"),
        "edge_matrix":  os.path.join(c.RILEY_DIR, "edge_matrix"),
        "transform_dir": c.TRANSFORM_DAY_DIR,
    },
}


def main():
    parser = argparse.ArgumentParser(
        description="Create DARP benchmark instances from transformed trip records.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--network",
        choices=["own", "riley"],
        default="own",
        help="Which network to use (default: own).",
    )
    parser.add_argument(
        "--folder",
        default=None,
        help=(
            "Output folder name under Data/ "
            "(default: Instances_<window>h-<start-hour>)."
        ),
    )
    parser.add_argument(
        "--start-hour",
        type=int,
        default=7,
        help="Start hour of the time window (default: 7).",
    )
    parser.add_argument(
        "--end-hour",
        type=int,
        default=9,
        help="End hour of the time window (default: 9).",
    )
    parser.add_argument(
        "--start-min",
        type=int,
        default=0,
        help="Start minute (default: 0).",
    )
    parser.add_argument(
        "--end-min",
        type=int,
        default=59,
        help="End minute (default: 59).",
    )
    parser.add_argument(
        "--capacity",
        type=int,
        default=4,
        help="Maximum passenger capacity per vehicle (default: 4).",
    )
    parser.add_argument(
        "--no-vehicles",
        action="store_true",
        help="Skip vehicle file generation.",
    )
    args = parser.parse_args()

    window_hours = args.end_hour - args.start_hour
    folder_name = args.folder or f"Instances_{window_hours}h-{args.start_hour}"
    instance_dir = os.path.join(c.DATA_DIR, folder_name) + "/"
    os.makedirs(instance_dir, exist_ok=True)

    net_cfg = _NETWORK_FILES[args.network]

    print(f"[04] Network      : {args.network}")
    print(f"[04] Time window  : {args.start_hour}:00 – {args.end_hour}:00")
    print(f"[04] Output folder: {instance_dir}")

    print("[04] Loading network …")
    network = District_Network()
    network.read_network_data(
        manhattan_geo_file=net_cfg["geo"],
        stops_geojson=net_cfg["stops_geojson"],
        edge_matrix_file=net_cfg["edge_matrix"],
    )

    vehicle_obj = Vehicle(len(network.districts))
    tests_data = []

    print("[04] Processing day files …")
    for file_name in uf.create_file_names():
        day_dataset = Dataset(file_name)
        average_requests = day_dataset.prepare_dataset(
            instance_dir,
            network=network,
            start_hr=args.start_hour,
            end_hr=args.end_hour,
            start_min=args.start_min,
            end_min=args.end_min,
            capacity=args.capacity,
            visualize=True,
            remove_same=True,
        )
        tests_data.append([
            "R" + str(day_dataset.nb_requests),
            day_dataset.file_name,
            day_dataset.nb_requests,
            day_dataset.nb_customers,
            average_requests,
        ])
        vehicle_obj.update_nb_trip_per_district(network=network, dataset=day_dataset)
        print(f"[04]   {file_name}: {day_dataset.nb_requests} requests")

    summary = pd.DataFrame(
        tests_data,
        columns=["Instance", "Dates", "No. requests", "No. customers", "average"],
    )
    summary.to_csv(instance_dir + "data.csv", index=False)
    print(f"[04] Instance summary → {instance_dir}data.csv")

    vehicle_obj.avg_trip_per_district()

    if not args.no_vehicles:
        print("[04] Generating vehicle files (proportional) …")
        create_vehicles_files_proportional(
            network=network,
            initial_vehicle=vehicle_obj,
            parent_dir=instance_dir,
        )
        print("[04] Generating vehicle files (uniform) …")
        create_vehicles_files_uniform(
            network=network,
            initial_vehicle=vehicle_obj,
            parent_dir=instance_dir,
        )

    print("[04] Done.")


if __name__ == "__main__":
    main()
