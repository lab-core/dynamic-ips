
from typing import Dict, List, Any
import constants as c
import numpy as np
import geopandas as gpd
from sklearn.neighbors import BallTree
import os
import json
import math
import hashlib
from datetime import datetime
from zoneinfo import ZoneInfo

MAX_STOP_DISTANCE_M = 1000.0
# Import your existing functions (assuming they're in a module called taxi_utils)
# from taxi_utils import _transform_record, _build_stop_index

EARTH_RADIUS_M = 6371008.8  # mean Earth radius in meters
NY_TZ = ZoneInfo("America/New_York")

def _to_epoch_seconds(datetime_str: str, tz: ZoneInfo = NY_TZ) -> int | None:
    """
    Convert datetime strings to epoch seconds (UTC).
    Handles two formats:
    - 2015 format: "2015 Jan 01 12:00:00 AM"
    - 2016 format: "2016-01-01T00:00:00.000"
    """
    if not datetime_str:
        return None

    try:
        # Check if it's the 2015 format (contains spaces and AM/PM)
        if ' ' in datetime_str and ('AM' in datetime_str or 'PM' in datetime_str):
            # 2015 format: "2015 Jan 01 12:00:00 AM"
            dt = datetime.strptime(datetime_str, "%Y %b %d %I:%M:%S %p")
            # Assume it's in NYC timezone
            dt = dt.replace(tzinfo=tz)
        else:
            # 2016 format: "2016-01-01T00:00:00.000"
            # Remove trailing Z if present and handle optional milliseconds
            clean_str = datetime_str.rstrip("Z")
            dt = datetime.fromisoformat(clean_str)
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=tz)
            else:
                dt = dt.astimezone(tz)

        return int(dt.timestamp())
    except Exception:
        return None

def _to_int(x, default=None):
    try:
        if x is None or (isinstance(x, str) and x.strip() == ""):
            return default
        return int(float(x))
    except Exception:
        return default

def _to_float(x, default=None):
    try:
        if x is None or (isinstance(x, str) and x.strip() == ""):
            return default
        return float(x)
    except Exception:
        return default

def _hash_id(parts: list[str], modulo: int = 10_000_000) -> int:
    """Stable 7-digit-ish numeric id derived from key fields."""
    seed = "|".join("" if p is None else str(p) for p in parts)
    h = hashlib.blake2b(seed.encode("utf-8"), digest_size=8).hexdigest()
    return int(h, 16) % modulo

def _nearest_stop_id(lat: float | None, lon: float | None, idx) -> int | None:
    if lat is None or lon is None or any(map(lambda v: v is None or math.isnan(v), [lat, lon])):
        return None
    try:
        return nearest_stop(lat, lon, index=idx)["stop_id"]
    except Exception:
        return None


def _transform_record(rec: dict, stop_index) -> dict:
    """
    Transform a raw NYC Taxi record into the requested schema.
    Handles both 2015 and 2016 data formats.
    """
    # Prefer lowercase keys if given, but tolerate the canonical Taxi names
    vendor = rec.get("vendorid", rec.get("VendorID"))
    ratecode = rec.get("ratecodeid", rec.get("RatecodeID"))
    paytype = rec.get("payment_type")

    # Handle different datetime field names by year
    # 2016 uses tpep_pickup_datetime/tpep_dropoff_datetime
    # 2015 uses pickup_datetime/dropoff_datetime
    pickup_iso = rec.get("tpep_pickup_datetime") or rec.get("pickup_datetime")
    dropoff_iso = rec.get("tpep_dropoff_datetime") or rec.get("dropoff_datetime")

    request_time = _to_epoch_seconds(pickup_iso)
    dropoff_time = _to_epoch_seconds(dropoff_iso)

    # coords
    o_lat = _to_float(rec.get("pickup_latitude"))
    o_lon = _to_float(rec.get("pickup_longitude"))
    d_lat = _to_float(rec.get("dropoff_latitude"))
    d_lon = _to_float(rec.get("dropoff_longitude"))

    # nearest virtual stop ids WITH distance filter
    origin_cell, o_dist = _nearest_stop_within(o_lat, o_lon, stop_index, MAX_STOP_DISTANCE_M)
    destination_cell, d_dist = _nearest_stop_within(d_lat, d_lon, stop_index, MAX_STOP_DISTANCE_M)

    # If either endpoint is outside Manhattan (too far from any stop), DROP the record
    if origin_cell is None or destination_cell is None:
        return None

    # numeric conversions
    out = {
        "tpep_dropoff_datetime": dropoff_time,
        "origin": {"lat": o_lat, "lon": o_lon} if o_lat is not None and o_lon is not None else None,
        "store_and_fwd_flag": rec.get("store_and_fwd_flag"),
        "payment_type": _to_int(paytype),
        "passenger_count": _to_int(rec.get("passenger_count")),
        "VendorID": _to_int(vendor),
        "total_amount": _to_float(rec.get("total_amount")),
        "mta_tax": _to_float(rec.get("mta_tax")),
        "extra": _to_float(rec.get("extra")),
        "destination": {"lat": d_lat, "lon": d_lon} if d_lat is not None and d_lon is not None else None,
        "request_time": request_time,
        "board_time_lb": request_time,  # as requested
        "tolls_amount": _to_float(rec.get("tolls_amount")),
        "improvement_surcharge": _to_float(rec.get("improvement_surcharge")),
        "RatecodeID": _to_int(ratecode),
        "destination_cell": destination_cell,
        "trip_distance": _to_float(rec.get("trip_distance")),
        "fare_amount": _to_float(rec.get("fare_amount")),
        "tip_amount": _to_float(rec.get("tip_amount")),
        "origin_cell": origin_cell,
        "creation_time": request_time,
    }

    # passenger_id / trip_id (deterministic, derived from stable fields)
    pid = _hash_id([
        vendor, pickup_iso, dropoff_iso,
        rec.get("pickup_longitude"), rec.get("pickup_latitude"),
        rec.get("dropoff_longitude"), rec.get("dropoff_latitude"),
        rec.get("total_amount"),
    ])
    out["passenger_id"] = pid
    out["trip_id"] = str(pid)

    return out


def _build_stop_index(stops_latlon_path=None):
    """
    Load your saved stops (lat/lon), ensure WGS84, and build a BallTree on (lat, lon) in radians.
    Returns a dict with the tree and the GeoDataFrame.
    """
    if stops_latlon_path is None:
        stops_latlon_path = os.path.join(c.STOP_DIR, "virtual_stops_latlon.geojson")
    else:
        stops_latlon_path = os.path.join(c.STOP_DIR, stops_latlon_path)

    if not os.path.exists(stops_latlon_path):
        raise FileNotFoundError(
            f"Couldn't find stops at {stops_latlon_path}. "
            "Run create_virtual_stops() first or pass a custom path."
        )

    stops_ll = gpd.read_file(stops_latlon_path)
    # Make sure we’re in EPSG:4326 (lat/lon degrees)
    if stops_ll.crs is None or stops_ll.crs.to_epsg() != 4326:
        stops_ll = stops_ll.to_crs(epsg=4326)

    # Build BallTree on (lat, lon) in radians (order: lat, lon!)
    lat = stops_ll.geometry.y.to_numpy()
    lon = stops_ll.geometry.x.to_numpy()
    coords_rad = np.deg2rad(np.c_[lat, lon])

    tree = BallTree(coords_rad, metric="haversine")
    return {"tree": tree, "stops": stops_ll}


def nearest_stop(lat, lon, index=None, stops_latlon_path=None, return_row=False):
    """
    Given a point (lat, lon) in degrees, return the nearest virtual stop.

    Parameters
    ----------
    lat : float
        Latitude in degrees.
    lon : float
        Longitude in degrees.
    index : dict, optional
        Output of _build_stop_index() for re-use across many queries.
        If not provided, we'll build it from 'stops_latlon_path'.
    stops_latlon_path : str, optional
        Path to virtual_stops_latlon.geojson (default uses c.DATA_DIR/stops/...).
    return_row : bool, optional
        If True, return the full GeoPandas row for the matched stop in addition to metadata.

    Returns
    -------
    dict (and optionally a GeoSeries)
        {
          "stop_id": int,
          "distance_m": float,
          "stop_lat": float,
          "stop_lon": float,
          "idx": int   # row index in the stops GeoDataFrame
        }
        If return_row=True, also returns the GeoSeries row as the second value.
    """
    if index is None:
        index = _build_stop_index(stops_latlon_path)

    tree = index["tree"]
    stops = index["stops"]

    # Query point in radians (order: lat, lon)
    q_rad = np.deg2rad(np.array([[lat, lon]]))

    dist_rad, ind = tree.query(q_rad, k=1)
    ridx = int(ind[0, 0])
    d_m = float(dist_rad[0, 0] * EARTH_RADIUS_M)

    row = stops.iloc[ridx]
    result = {
        "stop_id": int(row["stop_id"]),
        "distance_m": d_m,
        "stop_lat": float(row.geometry.y),
        "stop_lon": float(row.geometry.x),
        "idx": ridx,
    }
    if return_row:
        return result, row
    return result

def _nearest_stop_within(lat: float | None, lon: float | None, idx, max_dist_m: float) -> tuple[int | None, float | None]:
    """
    Return (stop_id, distance_m) if the nearest stop is within max_dist_m; otherwise (None, None).
    """
    if lat is None or lon is None or any(map(lambda v: v is None or math.isnan(v), [lat, lon])):
        return (None, None)
    try:
        res = nearest_stop(lat, lon, index=idx)  # uses BallTree + haversine
        if res["distance_m"] <= max_dist_m:
            return (int(res["stop_id"]), float(res["distance_m"]))
        return (None, None)
    except Exception:
        return (None, None)


def normalize_field_names(record: Dict[str, Any], year: int) -> Dict[str, Any]:
    """
    Normalize field names to match what _transform_record expects.
    Handles the differences between your actual data and the expected field names.
    """
    normalized = record.copy()

    if year == 2015:
        # Map your 2015 field names to what the code expects
        field_mapping = {
            'vendor_id': 'vendorid',  # Your data uses 'vendor_id', code expects 'vendorid'
            'imp_surcharge': 'improvement_surcharge',  # Your data uses 'imp_surcharge'
            # pickup_datetime and dropoff_datetime are already correct
        }
    else:  # 2016
        # Your 2016 data already matches the expected field names
        field_mapping = {}

    # Apply field name mappings
    for old_name, new_name in field_mapping.items():
        if old_name in normalized:
            normalized[new_name] = normalized[old_name]
            # Optionally remove the old field name
            # del normalized[old_name]

    return normalized


def transform_json_file(input_file: str, output_file: str, year: int, stop_index: Dict):
    """
    Transform a JSON file containing taxi records from your format to the target format.

    Parameters:
    - input_file: Path to input JSON file (one record per line or array of records)
    - output_file: Path to output JSON file
    - year: 2015 or 2016 (affects field name handling)
    - stop_index: Pre-built stop index from _build_stop_index()
    """

    transformed_records = []

    with open(input_file, 'r') as f:
        # Try to load as a single JSON array first
        try:
            f.seek(0)
            content = f.read()
            if content.strip().startswith('['):
                # JSON array format
                records = json.loads(content)
            else:
                # JSON Lines format (one record per line)
                f.seek(0)
                records = []
                for line in f:
                    line = line.strip()
                    if line:
                        records.append(json.loads(line))
        except json.JSONDecodeError:
            # Fall back to JSON Lines format
            f.seek(0)
            records = []
            for line in f:
                line = line.strip()
                if line:
                    records.append(json.loads(line))

    print(f"Processing {len(records)} records from {input_file}")

    for i, record in enumerate(records):
        try:
            # Normalize field names for your data format
            normalized_record = normalize_field_names(record, year)

            # Transform using the existing function
            transformed = _transform_record(normalized_record, stop_index)
            transformed_records.append(transformed)

            if (i + 1) % 1000 == 0:
                print(f"Processed {i + 1} records...")

        except Exception as e:
            print(f"Error processing record {i}: {e}")
            print(f"Problematic record: {record}")
            continue

    # Write transformed records
    with open(output_file, 'w') as f:
        json.dump(transformed_records, f, indent=2)

    print(f"Transformed {len(transformed_records)} records and saved to {output_file}")


def transform_multiple_files(input_dir: str, output_dir: str, stop_index: Dict):
    """
    Transform multiple JSON files in a directory.
    Automatically detects year from filename or content.
    """

    os.makedirs(output_dir, exist_ok=True)

    for filename in os.listdir(input_dir):
        if not filename.endswith('.json'):
            continue

        # Detect year from filename
        year = None
        if '2015' in filename:
            year = 2015
        elif '2016' in filename:
            year = 2016
        else:
            print(f"Cannot detect year for {filename}, skipping...")
            continue

        input_path = os.path.join(input_dir, filename)
        output_filename = f"transformed_{filename}"
        output_path = os.path.join(output_dir, output_filename)

        print(f"Transforming {filename} (year: {year})...")
        transform_json_file(input_path, output_path, year, stop_index)

