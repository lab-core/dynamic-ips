"""
Download raw NYC TLC taxi trip records from the Socrata open-data API.

Records are fetched day-by-day for the dates configured in constants.py
(DATES_2015 + DATES_2016).  Each day is saved as a single JSON file in
Data/days/.  The same day files are used for both the custom network and
Riley's network in the next pipeline step (03_transform_trips.py).

Usage
-----
    python scripts/02_fetch_trips.py                   # fetch all configured dates
    python scripts/02_fetch_trips.py --year 2015       # only 2015 dates
    python scripts/02_fetch_trips.py --year 2016       # only 2016 dates
    python scripts/02_fetch_trips.py --dates 2015-07-06 2016-01-09

Environment
-----------
    SODA_APP_TOKEN  (optional) — Socrata API token to raise rate limits.
    Set with:  export SODA_APP_TOKEN=<your_token>
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Simulation.get_trip_records import fetch_data_for_date
import constants as c

ALL_DATES = (
    [f"2015-{m}" for m in c.DATES_2015] +
    [f"2016-{m}" for m in c.DATES_2016]
)


def main():
    parser = argparse.ArgumentParser(
        description="Download NYC TLC taxi records from the Socrata API.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--year",
        choices=["2015", "2016"],
        default=None,
        help="Fetch only dates for a specific year (default: both years).",
    )
    parser.add_argument(
        "--dates",
        nargs="+",
        metavar="YYYY-MM-DD",
        default=None,
        help="Explicit list of dates to fetch (overrides --year).",
    )
    args = parser.parse_args()

    if args.dates:
        dates = args.dates
    elif args.year == "2015":
        dates = [f"2015-{m}" for m in c.DATES_2015]
    elif args.year == "2016":
        dates = [f"2016-{m}" for m in c.DATES_2016]
    else:
        dates = ALL_DATES

    print(f"[02] Fetching {len(dates)} day(s) → {c.DAY_DIR}/")
    for date in dates:
        print(f"[02]   {date} …")
        fetch_data_for_date(date_str=date)

    print(f"[02] Done — {len(dates)} file(s) saved to {c.DAY_DIR}/")


if __name__ == "__main__":
    main()
