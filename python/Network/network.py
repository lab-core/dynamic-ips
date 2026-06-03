import networkx as nx
import pandas as pd
import Visualization.visualize_network as ivf
import constants as c
import json
import numpy as np
from typing import Optional

from Network.district import District
from Visualization.plot_config import PlotConfig


class District_Network(object):
    """
    The `Network` class serves as a structure for storing network topology and geographic data.

    Attributes
    ----------
    districts : list
        List of district identifiers in the network.
    durations : list
        Duration values for network connections.
    cell_to_district : dict
        Mapping from cell IDs to their parent district IDs.
    cell_to_latitude : dict
        Mapping from cell IDs to latitude coordinates.
    cell_to_longitude : dict
        Mapping from cell IDs to longitude coordinates.
    locations : numpy.ndarray
        Array of 3D location coordinates (N x 3 shape).
    limited_districts : list
        List of districts with connection limitations.
    centers : list
        List of network center points or hub locations.
    graph : networkx.DiGraph
        Directed graph representing the network topology.
    nodes : list
        List of all nodes in the network.
    """

    def __init__(self, districts=None, durations=None, cell_to_district=None, cell_to_latitude=None,
                 cell_to_longitude=None, locations=None):
        """
        Initialize a Network instance with optional parameters.
        """
        # Initialize input parameters with safe defaults to avoid mutable default argument issues
        if districts is None:
            districts = []
        if durations is None:
            durations = []
        if cell_to_district is None:
            cell_to_district = {}
        if cell_to_latitude is None:
            cell_to_latitude = {}
        if cell_to_longitude is None:
            cell_to_longitude = {}
        if locations is None:
            locations = np.empty(shape=[0, 3])

        self.districts = districts
        self.durations = durations
        self.cell_to_district = cell_to_district
        self.cell_to_latitude = cell_to_latitude
        self.cell_to_longitude = cell_to_longitude
        self.locations = locations
        self.limited_districts = []
        self.centers = []
        self.graph = nx.DiGraph()
        self.nodes = []

    def __str__(self):
        """
        Return a string representation of the Network object.

        Returns:
            str: Formatted string showing all attributes and their values.
        """
        class_string = str(self.__class__) + ": {"
        for attribute, value in self.__dict__.items():
            class_string += str(attribute) + ": " + str(value) + ",\n"
        class_string += "}"
        return class_string

    def read_district_data(self, manhattan_geo_file):
        """
        Read NYC taxi zones GeoJSON and populate self.districts with District objects.
        - Supports Polygon and MultiPolygon.
        - Uses exterior rings only (ignores holes).
        - Converts (lon, lat) -> (lat, lon).
        - For MultiPolygons, creates one District per exterior ring (suffixes the name with '#part').
        """
        self.districts = []  # reset if re-reading

        path = f"{c.TAXI_ZONE_DIR}/{manhattan_geo_file}.geojson"
        with open(path, "r") as f:
            manhattan_geo = json.load(f)

        features = manhattan_geo.get("features", [])
        for feat in features:
            props = feat.get("properties", {})
            geom = feat.get("geometry", {})
            gtype = geom.get("type")
            coords = geom.get("coordinates")

            if not gtype or coords is None:
                continue

            district_id = props.get("LocationID")
            district_name = props.get("zone", "unknown")

            # Collect all exterior rings as lists of (lat, lon)
            exterior_rings_latlon = []

            if gtype == "Polygon":
                # coords: [ exterior, hole1, hole2, ... ]
                if len(coords) >= 1:
                    exterior = coords[0]  # list of [lon, lat]
                    ring = np.asarray(exterior, dtype=float)
                    # swap (lon,lat) -> (lat,lon)
                    ring = ring[:, [1, 0]]
                    exterior_rings_latlon.append(ring)

            elif gtype == "MultiPolygon":
                # coords: [ [ polygon1_rings... ], [ polygon2_rings... ], ... ]
                for poly in coords:
                    if len(poly) >= 1:
                        exterior = poly[0]  # first ring is exterior
                        ring = np.asarray(exterior, dtype=float)
                        ring = ring[:, [1, 0]]  # (lat,lon)
                        exterior_rings_latlon.append(ring)

            else:
                # Skip non-polygonal geometries
                continue

            # Create a District for each exterior ring
            for idx, ring in enumerate(exterior_rings_latlon, start=1):
                name = district_name if len(exterior_rings_latlon) == 1 else f"{district_name} #{idx}"
                district = District(district_id, name, ring)
                self.districts.append(district)

    def update_cells_from_geojson(self, stops_geojson: str):
        """
        Load cells from a GeoJSON of Point features and assign each to a District.

        Expected feature format:
          {
            "type": "Feature",
            "properties": { "stop_id": <int>, ... },
            "geometry": { "type": "Point", "coordinates": [lon, lat] }
          }

        Effects:
          - self.locations: np.array([[stop_id, lat, lon], ...]) sorted by stop_id
          - self.cell_to_latitude[stop_id] = lat
          - self.cell_to_longitude[stop_id] = lon
          - self.cell_to_district[stop_id] = district.cartodb_id   (only for assigned cells)
          - Each District.cells becomes an np.array of its assigned cells

        Returns:
          - unassigned: list of [stop_id, lat, lon] that didn’t match any district
        """

        # Reset per-run maps
        self.cell_to_latitude.clear()
        self.cell_to_longitude.clear()
        self.cell_to_district.clear()

        # Clear district cell assignments
        for d in self.districts:
            d.cells = []  # will convert to np.array later

        # --- Load GeoJSON ---
        path = f"{c.STOP_DIR}/{stops_geojson}.geojson"
        with open(path, "r") as f:
            data = json.load(f)

        # Support FeatureCollection, list of Features, or single Feature
        if isinstance(data, dict) and data.get("type") == "FeatureCollection":
            features = data.get("features", [])
        elif isinstance(data, list):
            features = data
        else:
            features = [data] if isinstance(data, dict) else []

        # Build cell records: [stop_id, lat, lon]
        cells = []
        for feat in features:
            props = feat.get("properties", {}) or {}
            geom = feat.get("geometry", {}) or {}
            if geom.get("type") != "Point":
                continue
            coords = geom.get("coordinates")
            stop_id = props.get("stop_id")
            if coords is None or stop_id is None:
                continue
            lon, lat = float(coords[0]), float(coords[1])  # GeoJSON is [lon, lat]
            stop_id = int(stop_id)

            cells.append([stop_id, lat, lon])
            self.cell_to_latitude[stop_id] = lat
            self.cell_to_longitude[stop_id] = lon

        # Stable ordering
        cells.sort(key=lambda x: x[0])
        self.locations = np.asarray(cells, dtype="double")

        # --- Assign cells to districts ---
        unassigned = []
        for stop_id, lat, lon in cells:
            assigned = False
            for district in self.districts:
                if district.point_inside_district(lat, lon):  # uses your corrected method
                    district.cells.append([stop_id, lat, lon])
                    self.cell_to_district[stop_id] = district.cartodb_id
                    assigned = True
                    break
            if not assigned:
                unassigned.append([stop_id, lat, lon])

        # Finalize arrays on each district
        for district in self.districts:
            district.cells = np.asarray(district.cells) if district.cells else np.empty((0, 3), dtype="double")

        return unassigned

    def read_duration_matrix(self, edge_matrix_file):

        path = f"{c.STOP_DIR}/{edge_matrix_file}.json"
        with open(path, "r") as f:
            data = json.load(f)


        # Build long-form rows
        rows = []
        for p in data["pairs"]:
            oi = int(p["origin_id"])
            di = int(p["destination_id"])
            dur = float(p.get("duration_s", np.nan))
            dist = float(p.get("distance_m", np.nan))
            rows.append([oi, di, dur, dist])

        df = pd.DataFrame(rows, columns=["startID", "EndID", "duration", "distance_m"])
        # Ensure integer IDs and sorted order (optional, but nice)
        df["startID"] = df["startID"].astype(int)
        df["EndID"] = df["EndID"].astype(int)
        df.sort_values(["startID", "EndID"], inplace=True, kind="stable", ignore_index=True)

        # Expose both tables
        self.durations = df[["startID", "EndID", "duration"]].copy()

    def compute_region_centers(self):
        """
        Build self.centers as a DataFrame mapping each region's Zone_ID (cartodb_id)
        to the Location_ID of the cell closest to that region's (lon, lat) centroid.

        Assumes region.cells is an ndarray with columns: [id, lon, lat].
        """
        centers = []

        for region in self.districts:
            cells = getattr(region, "cells", None)
            if cells is None or cells.size == 0:
                continue

            # Extract lon/lat as float (guards against mixed dtypes)
            xy = cells[:, 1:3].astype(float)
            # Optional: drop rows with NaNs
            mask = ~np.isnan(xy).any(axis=1)
            if not np.any(mask):
                continue
            xy = xy[mask]
            ids = cells[mask, 0]

            # Centroid of lon/lat (Euclidean in degrees—fine for small areas)
            centroid = xy.mean(axis=0)  # shape (2,)

            # Find nearest cell to centroid (no sqrt needed for argmin)
            idx = np.argmin(((xy - centroid) ** 2).sum(axis=1))
            centroid_id = int(ids[idx])

            centers.append([region.cartodb_id, centroid_id])

        self.centers = pd.DataFrame(centers, columns=["Zone_ID", "Location_ID"])

    def read_network_data(self, manhattan_geo_file, stops_geojson=None,
                          edge_matrix_file=None, make_plot=True, plot_config: Optional[PlotConfig] = None):
        self.read_district_data(manhattan_geo_file)
        if stops_geojson is not None:
            self.update_cells_from_geojson(stops_geojson)
        if edge_matrix_file is not None:
            self.read_duration_matrix(edge_matrix_file)

        self.compute_region_centers()
        root_file = c.TAXI_ZONE_DIR + "/" + "Zones.csv"
        txt_file = c.TAXI_ZONE_DIR + "/" + "Zones.txt"
        self.centers.to_csv(root_file, index=False)

        file = open(txt_file, "w")
        file.write("COLUMNS\n\n")

        for col in self.centers.columns:
            file.write(col)
            file.write("\n")
        file.write("\nZONE_INFO\n")
        df_as_string = self.centers.to_string(header=False, index=False)
        file.write(df_as_string)
        file.close()
        map_folder = f"{c.DATA_DIR}/{"maps/"}"
        if make_plot:
            if plot_config is None:
                plot_config = PlotConfig()
            ivf.plot_districtIDs(district_network=self, parent_folder=map_folder, config=plot_config)
            ivf.plot_centers(district_network=self, parent_folder=map_folder, config=plot_config)
            ivf.plot_map_cells(district_network=self, parent_folder=map_folder, config=plot_config)
