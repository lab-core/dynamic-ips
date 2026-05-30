from pathlib import Path
from typing import Optional

import geopandas as gpd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

import constants as c
from Visualization.plot_config import PlotConfig
import contextily as cx
import os
import osmnx as ox
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset

def plot_virtual_stops(output_filename: str = "virtual_stops.pdf", input_stop_file: str = "virtual_stops.geojson",
                       zoom_bounds: tuple = None, stop_color='steelblue'):
    """
    Plot the virtual stops that were created and saved by create_virtual_stops().
    Reads the stops from the 'stops' folder and creates a visualization with a zoomed inset.
    """
    # Load the saved stops
    stops_folder = c.DATA_DIR + "/stops"
    stops_file = os.path.join(stops_folder, input_stop_file)

    if not os.path.exists(stops_file):
        raise FileNotFoundError(f"Stops file not found at {stops_file}. Run create_virtual_stops() first.")

    stops = gpd.read_file(stops_file)

    # Download the network again for plotting (could be optimized by saving/loading)
    G = ox.graph_from_place(c.PLACE, network_type=c.NETWORK_TYPE, simplify=True)
    G = ox.project_graph(G)

    # 6) Plot: base = road network, overlay = stop points
    fig, ax = ox.plot_graph(
        G,
        node_size=0,
        edge_color="0.6",
        edge_linewidth=0.5,
        bgcolor="white",
        show=False,
        close=False,
    )
    stops.plot(ax=ax, markersize=2.5, alpha=0.9, color=stop_color)  # default color so it's easy to tweak

    # Create zoomed inset axes
    if zoom_bounds is not None:
        zoom_bounds = (586500, 589500, 4516000, 4519000)
        xmin, xmax, ymin, ymax = zoom_bounds
        axins = zoomed_inset_axes(ax, zoom=1.6, loc='upper left', borderpad=2)

        # Plot the same data in the inset
        ox.plot_graph(
            G,
            ax=axins,
            node_size=0,
            edge_color="#999999",
            edge_linewidth=0.5,
            bgcolor="white",
            show=False,
            close=False,
        )
        stops.plot(ax=axins, markersize=6, alpha=0.9, color=stop_color)  # slightly larger markers in inset

        # Set the zoom bounds for the inset
        axins.set_xlim(xmin, xmax)
        axins.set_ylim(ymin, ymax)

        # Remove tick labels from inset
        axins.set_xticks([])
        axins.set_yticks([])

        # Draw connection lines from main plot to inset
        mark_inset(ax, axins, loc1=1, loc2=3, fc="none", ec="0.5")

        for spine in axins.spines.values():
            spine.set_visible(True)  # ensures spines are drawn
            spine.set_edgecolor("0.5")
            spine.set_linewidth(1)

    plt.tight_layout()

    # Save
    maps_dir = os.path.join(c.DATA_DIR, "maps")
    os.makedirs(maps_dir, exist_ok=True)
    figure_path = os.path.join(maps_dir, output_filename)
    plt.tight_layout()
    plt.savefig(figure_path, bbox_inches="tight")
    plt.close()

    return figure_path


def plot_taxi_zone(add_basemap=True):
    # Input shapefile & output GeoJSON path
    shp_path = Path(c.TAXI_ZONE_DIR) / "taxi_zones.shp"

    # Read shapefile
    row_gdf = gpd.read_file(shp_path)

    # The NYC TLC taxi zones shapefile is usually EPSG:2263 (feet).
    # Reproject to Web Mercator to plot on map.

    # Filter only Manhattan borough
    manhattan = row_gdf[row_gdf["borough"] == "Manhattan"]

    manhattan_wm = manhattan.to_crs(3857)
    ax = manhattan_wm.plot(edgecolor="black", facecolor="steelblue", linewidth=0.7, alpha=0.4)
    if add_basemap:
        cx.add_basemap(ax, source=cx.providers.CartoDB.Positron)
    ax.set_axis_off()

    plt.tight_layout()
    folder_path = os.path.join(c.DATA_DIR, 'maps')
    os.makedirs(folder_path, exist_ok=True)
    figure_path = os.path.join(folder_path, 'taxi_zones.pdf')
    plt.savefig(figure_path, bbox_inches="tight")
    plt.close()


def plot_districts(district_network, mapsize=None, config: Optional[PlotConfig] = None):
    if config is None:
        config = PlotConfig()
    if mapsize is None:
        mapsize = config.district_map_size
    fig, ax = plt.subplots(figsize=mapsize)
    ax.set_aspect('equal')
    for region in district_network.districts:
        latitude = region.coordinates[:, 0]
        longitude = region.coordinates[:, 1]
        plt.plot(longitude, latitude, 'k', linewidth=0.7)

    ax.yaxis.set_major_locator(MultipleLocator(0.04))
    ax.xaxis.set_major_locator(MultipleLocator(0.04))
    plt.xlabel('Longitude', fontsize=config.axis_label_fsize, fontweight='bold')
    plt.ylabel('Latitude', fontsize=config.axis_label_fsize, fontweight='bold')
    plt.tick_params(axis='both', labelsize=config.tick_label_fsize)

    plt.tight_layout()
    return fig, ax


def plot_districtIDs(district_network, parent_folder, config: Optional[PlotConfig] = None):
    if config is None:
        config = PlotConfig()
    fig, ax = plot_districts(district_network, config=config)

    # Collect all region info first
    region_info = []
    for region in district_network.districts:
        if region.cells.size == 0:
            continue

        # Compute centroid coordinates
        centroid_lon = region.cells[:, 1].mean()
        centroid_lat = region.cells[:, 2].mean()

        # Compute squared distances and find index of closest cell
        dists_sq = (region.cells[:, 1] - centroid_lon) ** 2 + (region.cells[:, 2] - centroid_lat) ** 2
        closest_idx = np.argmin(dists_sq)

        # Coordinates for placing the label
        text_y = region.cells[closest_idx, 1]
        text_x = region.cells[closest_idx, 2]

        plt.text(text_x, text_y, region.cartodb_id, fontsize=config.region_fsize)
        region_info.append(f"{region.cartodb_id} - {region.name}")

    # Create two-column text layout
    if region_info:
        mid_point = (len(region_info) + 1) // 2  # +1 to handle odd numbers
        col1 = region_info[:mid_point]
        col2 = region_info[mid_point:]

        # Pad the shorter column with empty strings to align properly
        max_rows = max(len(col1), len(col2))
        col1.extend([''] * (max_rows - len(col1)))
        col2.extend([''] * (max_rows - len(col2)))

        # Create formatted text with consistent spacing
        text_lines = []
        for i in range(max_rows):
            # Adjust the spacing (40 characters) based on your needs
            line = f"{col1[i]:<40}{col2[i]}"
            text_lines.append(line)

        text_str = '\n'.join(text_lines)

        # You might need to adjust the x-position (1.03) to accommodate wider text
        ax.text(1.03, 0.99, text_str, transform=ax.transAxes,
                fontsize=config.map_legend_fsize,
                verticalalignment='top',
                fontfamily='monospace')  # Monospace for better alignment

    ax.set_axis_off()

    image_dir = parent_folder
    file_name = "Districts.pdf"
    os.makedirs(image_dir, exist_ok=True)
    fig.savefig(image_dir + file_name, bbox_inches="tight")
    plt.close(fig)


def plot_centers(district_network, parent_folder, config: Optional[PlotConfig] = None):
    if config is None:
        config = PlotConfig()
    fig, ax = plot_districts(district_network, config=config)
    for region in district_network.districts:
        if region.cells.size == 0:
            continue

            # Compute centroid coordinates
        centroid_lon = region.cells[:, 1].mean()
        centroid_lat = region.cells[:, 2].mean()

        # Find the index of the closest cell to the centroid
        dists_sq = (region.cells[:, 1] - centroid_lon) ** 2 + (region.cells[:, 2] - centroid_lat) ** 2
        closest_idx = np.argmin(dists_sq)

        # Plot the centroid point
        x, y = region.cells[closest_idx, 2], region.cells[closest_idx, 1]
        ax.scatter(x, y, s=1, color='red')
    ax.set_axis_off()
    image_dir = parent_folder
    file_name = "Centers.pdf"
    os.makedirs(image_dir, exist_ok=True)
    fig.savefig(image_dir + file_name, bbox_inches="tight")
    plt.close(fig)


def plot_map_cells(district_network, parent_folder, config: Optional[PlotConfig] = None):
    if config is None:
        config = PlotConfig()
    fig, ax = plot_districts(district_network, config=config)

    for region in district_network.districts:
        if len(region.cells):
            plt.scatter(region.cells[:, 2], region.cells[:, 1], c="blue", s=1)
    ax.set_axis_off()
    image_dir = parent_folder
    file_name = "Map_Cells.pdf"
    os.makedirs(image_dir, exist_ok=True)
    fig.savefig(image_dir + file_name, bbox_inches="tight")
    plt.close(fig)



"""
Example:
----------
    plot_virtual_stops(output_filename="rely_virtual_stops_with_zoom.pdf", input_stop_file="riley_virtual_stops.geojson",
                       zoom_bounds=(585000, 588000, 4513000, 4516000), stop_color='maroon')

    plot_virtual_stops(output_filename="virtual_stops_with_zoom.pdf", input_stop_file="virtual_stops.geojson",
                       zoom_bounds=(585000, 588000, 4513000, 4516000))
    plot_virtual_stops()
"""