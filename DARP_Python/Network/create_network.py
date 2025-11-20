from pathlib import Path
import geopandas as gpd
import constants as c
import os
import numpy as np
from shapely.geometry import Point
from sklearn.neighbors import BallTree
import osmnx as ox
import json
import requests
from typing import List, Tuple, Dict, Union

def create_taxi_zone_shapefile():
    # Input shapefile & output GeoJSON path
    shp_path = Path(c.TAXI_ZONE_DIR) / "taxi_zones.shp"
    out_path = Path(c.TAXI_ZONE_DIR) / "taxi_zones.geojson"

    # Read shapefile
    row_gdf = gpd.read_file(shp_path)

    # Filter only Manhattan borough
    manhattan = row_gdf[row_gdf["borough"] == "Manhattan"]

    # The NYC TLC taxi zones shapefile is usually EPSG:2263 (feet).
    # Reproject to WGS84 (lon/lat) so the JSON has standard coordinates.
    manhattan_coord = manhattan.to_crs(4326)

    # keep only useful columns
    keep_cols = ["LocationID", "zone"]
    keep_cols = [c for c in keep_cols if c in manhattan_coord.columns]
    if keep_cols:
        manhattan_coord = manhattan_coord[keep_cols + ["geometry"]]

    # Write as GeoJSON
    manhattan_coord.to_file(out_path, driver="GeoJSON")
    print(f"Saved GeoJSON with geometries to: {out_path}")


def create_virtual_stops(exclude_highway=True):
    """
        Create virtual stop points and save them to GeoJSON files in c.DATA_DIR/'stops'.
        """

    # 1) Download drivable network & project to meters
    G = ox.graph_from_place(c.PLACE, network_type=c.NETWORK_TYPE, simplify=True)
    G = ox.project_graph(G)
    nodes, edges = ox.graph_to_gdfs(G)

    # 2) Candidate "corners": true intersections only
    # (street_count >= 3 catches T and + junctions; endpoints/cul-de-sacs are dropped)
    intersections = nodes[nodes["street_count"] >= 3].copy()

    # 3) Optionally remove intersections near highways/ramps/limited access
    if exclude_highway:
        def has_any(vals, targets):
            if isinstance(vals, list):
                return any(v in targets for v in vals)
            return vals in targets

        bad_kinds = {"motorway", "trunk", "motorway_link", "trunk_link"}
        restricted = edges[edges["highway"].apply(lambda v: has_any(v, bad_kinds))]
        if not restricted.empty:
            restricted_buffer = restricted.buffer(c.HIGHWAY_BUFFER_M).union_all()
            intersections = intersections[~intersections.intersects(restricted_buffer)]

    # 4) Thinning: keep a Poisson-disk subset with min Euclidean spacing
    xy = np.column_stack([intersections.geometry.x, intersections.geometry.y])
    tree = BallTree(xy)
    np.random.seed(0)
    order = np.random.permutation(len(xy))  # randomize so we don’t favor any one area
    keep = np.zeros(len(xy), dtype=bool)
    removed = np.zeros(len(xy), dtype=bool)

    for i in order:
        if removed[i]:
            continue
        keep[i] = True
        # mark all neighbors within MIN_STOP_SPACING_M as removed
        nbrs = tree.query_radius(xy[i].reshape(1, -2), r=c.MIN_STOP_SPACING_M)[0]
        removed[nbrs] = True
        removed[i] = False  # re-enable the one we kept

    stops = intersections.iloc[keep].copy()
    stops["stop_id"] = range(0, len(stops))

    # 5) Save for solver (and GIS) into DATA_DIR / 'stops'
    stops_dir = os.path.join(c.DATA_DIR, "stops")
    os.makedirs(stops_dir, exist_ok=True)

    stops_projected_path = os.path.join(stops_dir, "virtual_stops.geojson")
    stops_ll_path = os.path.join(stops_dir, "virtual_stops_latlon.geojson")

    stops[["stop_id", "geometry"]].to_file(stops_projected_path, driver="GeoJSON")
    stops_ll = stops.to_crs(epsg=4326)
    stops_ll[["stop_id", "geometry"]].to_file(stops_ll_path, driver="GeoJSON")

    # Return paths in case the caller wants them
    return {
        "projected_path": stops_projected_path,
        "latlon_path": stops_ll_path,
        "crs_projected": stops.crs,
    }


def create_riley_virtual_stops():
    """
    Read Riley's location_to_cell.json (keys: "lat,lon" -> value: cell_id),
    build stop points, and save to GeoJSON in the same format as create_virtual_stops,
    using filenames:
      - riley_virtual_stops.geojson (projected)
      - riley_virtual_stops_latlon.geojson (EPSG:4326)
    Returns a dict with paths and projected CRS.
    """
    # 1) Load JSON
    loc2cell_path = os.path.join(c.RILEY_DIR, "location_to_cell.json")
    with open(loc2cell_path, "r") as f:
        loc2cell = json.load(f)

    if not isinstance(loc2cell, dict) or not loc2cell:
        raise ValueError("location_to_cell.json is empty or not a mapping.")

    # 2) Parse coordinates into GeoDataFrame (lat/lon -> EPSG:4326)
    coords = []
    cell_ids = []
    for k, v in loc2cell.items():
        # keys look like "lat,lon"
        try:
            lat_str, lon_str = k.split(",")
            lat = float(lat_str.strip())
            lon = float(lon_str.strip())
        except Exception as e:
            raise ValueError(f"Bad coordinate key '{k}': {e}")
        coords.append(Point(lon, lat))
        cell_ids.append(v)

    stops_ll = gpd.GeoDataFrame(
        {
            "stop_id": cell_ids,
            "cell_id": cell_ids,  # kept for reference; not written to solver files
        },
        geometry=coords,
        crs="EPSG:4326",
    )

    # Sort by stop_id before projecting
    stops_ll = stops_ll.sort_values("stop_id").reset_index(drop=True)

    # 3) Choose a sensible projected CRS (local UTM based on data extent)
    try:
        projected_crs = ox.project_graph(ox.graph_from_place(c.PLACE, network_type=c.NETWORK_TYPE)).graph['crs']

    except Exception:
        # Fallback if osmnx can't infer UTM; Web Mercator is widely supported
        projected_crs = "EPSG:3857"

    stops = stops_ll.to_crs(projected_crs)

    # 4) Save for solver (and GIS) into DATA_DIR / 'stops'
    stops_dir = os.path.join(c.DATA_DIR, "stops")
    os.makedirs(stops_dir, exist_ok=True)

    stops_projected_path = os.path.join(stops_dir, "riley_virtual_stops.geojson")
    stops_ll_path = os.path.join(stops_dir, "riley_virtual_stops_latlon.geojson")

    # Match the schema of your previous function: only ["stop_id", "geometry"]
    stops[["stop_id", "geometry"]].to_file(stops_projected_path, driver="GeoJSON")
    stops_ll[["stop_id", "geometry"]].to_file(stops_ll_path, driver="GeoJSON")

    return {
        "projected_path": stops_projected_path,
        "latlon_path": stops_ll_path,
        "crs_projected": stops.crs,
    }


def create_riley_virtual_stops_from_real_points():
    """
    Read Riley's real_location_matrix.json (list of "lat,lon" strings),
    build stop points with stop_id starting from 0, and save to GeoJSON in the same format as create_virtual_stops,
    using filenames:
      - riley_virtual_stops.geojson (projected)
      - riley_virtual_stops_latlon.geojson (EPSG:4326)
    Returns a dict with paths and projected CRS.
    """
    # 1) Load JSON
    loc_matrix_path = os.path.join(c.RILEY_DIR, "real_location_matrix.json")
    with open(loc_matrix_path, "r") as f:
        location_list = json.load(f)

    if not isinstance(location_list, list) or not location_list:
        raise ValueError("real_location_matrix.json is empty or not a list.")

    # 2) Parse coordinates into GeoDataFrame (lat/lon -> EPSG:4326)
    coords = []
    stop_ids = []

    for idx, coord_str in enumerate(location_list):
        # Each item looks like "40.70328140258789,-74.0171890258789" (lat,lon format)
        try:
            lat_str, lon_str = coord_str.split(",")
            lat = float(lat_str.strip())
            lon = float(lon_str.strip())
        except Exception as e:
            raise ValueError(f"Bad coordinate string '{coord_str}' at index {idx}: {e}")

        coords.append(Point(lon, lat))  # Point constructor needs (longitude, latitude)
        stop_ids.append(idx)  # stop_id starts from 0

    stops_ll = gpd.GeoDataFrame(
        {
            "stop_id": stop_ids,
            "cell_id": stop_ids,  # kept for reference; not written to solver files
        },
        geometry=coords,
        crs="EPSG:4326",
    )

    # Sort by stop_id before projecting (already in order, but keeping consistency)
    stops_ll = stops_ll.sort_values("stop_id").reset_index(drop=True)

    # 4) Save lat/lon file BEFORE projection
    stops_dir = os.path.join(c.DATA_DIR, "stops")
    os.makedirs(stops_dir, exist_ok=True)
    stops_ll_path = os.path.join(stops_dir, "riley_virtual_stops_latlon.geojson")

    # Save original lat/lon coordinates
    stops_ll[["stop_id", "geometry"]].to_file(stops_ll_path, driver="GeoJSON")

    # 3) Now apply projection
    # Choose a sensible projected CRS (local UTM based on data extent)
    try:
        projected_crs = ox.project_graph(ox.graph_from_place(c.PLACE, network_type=c.NETWORK_TYPE)).graph['crs']

    except Exception:
        # Fallback if osmnx can't infer UTM; Web Mercator is widely supported
        projected_crs = "EPSG:3857"

    stops = stops_ll.to_crs(projected_crs)

    # 5) Save projected file
    stops_projected_path = os.path.join(stops_dir, "riley_virtual_stops.geojson")

    # Save projected coordinates
    stops[["stop_id", "geometry"]].to_file(stops_projected_path, driver="GeoJSON")

    return {
        "projected_path": stops_projected_path,
        "latlon_path": stops_ll_path,
        "crs_projected": stops.crs,
    }


def save_high_precision_geojson(gdf, filepath):
    """Save GeoDataFrame as GeoJSON with full coordinate precision"""
    features = []
    for idx, row in gdf.iterrows():
        geom = row.geometry
        feature = {
            "type": "Feature",
            "properties": {"stop_id": row["stop_id"]},
            "geometry": {
                "type": "Point",
                "coordinates": [geom.x, geom.y]  # lon, lat
            }
        }
        features.append(feature)

    geojson_data = {
        "type": "FeatureCollection",
        "features": features
    }

    with open(filepath, 'w') as f:
        json.dump(geojson_data, f, separators=(',', ':'))
"""
Example:
----------
    create_taxi_zone_shapefile()
    create_virtual_stops()
    create_riley_virtual_stops()
    create_riley_virtual_stops_from_real_points()
     
"""
