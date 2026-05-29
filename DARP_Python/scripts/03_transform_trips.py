"""
Transform raw NYC taxi day records into the standard schema used by the solver.

For each raw JSON file in the input directory the script:
  - normalises field names across the 2015 and 2016 data formats
  - assigns origin/destination virtual stop IDs (nearest-stop lookup)
  - drops records whose endpoints lie more than 1 000 m from any stop
  - writes a cleaned JSON file per day

The --network flag controls which set of virtual stops is used for the
nearest-stop lookup:

  own    — custom network (virtual_stops_latlon.geojson in Data/stops/)
  riley  — Riley et al. network (riley_virtual_stops_latlon.geojson in
            Data/manhattan-network/, downloaded from Zenodo)

Usage
-----
    python scripts/03_transform_trips.py --network own    # custom network
    python scripts/03_transform_trips.py --network riley  # Riley's network

    # Override I/O directories:
    python scripts/03_transform_trips.py --network own \\
        --input-dir  Data/days/ \\
        --output-dir Data/transform_days/
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Simulation.transform_day_inputs import _build_stop_index, transform_multiple_files
import constants as c

# Stop-file paths per network
_STOP_FILES = {
    "own":   os.path.join(c.STOP_DIR,  "virtual_stops_latlon.geojson"),
    "riley": os.path.join(c.RILEY_DIR, "riley_virtual_stops_latlon.geojson"),
}


def main():
    parser = argparse.ArgumentParser(
        description="Transform raw NYC taxi records: assign virtual stop IDs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--network",
        choices=["own", "riley"],
        default="own",
        help="Which virtual-stop network to use (default: own).",
    )
    parser.add_argument(
        "--input-dir",
        default=c.DAY_DIR,
        help=f"Directory with raw day JSON files (default: {c.DAY_DIR}).",
    )
    parser.add_argument(
        "--output-dir",
        default=c.TRANSFORM_DAY_DIR,
        help=f"Directory for transformed output files (default: {c.TRANSFORM_DAY_DIR}).",
    )
    args = parser.parse_args()

    stops_path = _STOP_FILES[args.network]
    if not os.path.exists(stops_path):
        print(f"[03] ERROR: stop file not found: {stops_path}")
        if args.network == "riley":
            print(
                "[03]   Download riley_virtual_stops_latlon.geojson from\n"
                "       https://doi.org/10.5281/zenodo.18745880\n"
                f"       and place it in {c.RILEY_DIR}/"
            )
        else:
            print("[03]   Run scripts/01_build_network.py --steps stops first.")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"[03] Network : {args.network}")
    print(f"[03] Stops   : {stops_path}")
    print(f"[03] Input   : {args.input_dir}")
    print(f"[03] Output  : {args.output_dir}")

    print("[03] Building stop spatial index …")
    stop_index = _build_stop_index(stops_latlon_path=stops_path)
    print("[03] Transforming trip files …")
    transform_multiple_files(args.input_dir, args.output_dir, stop_index)
    print("[03] Done.")


if __name__ == "__main__":
    main()
