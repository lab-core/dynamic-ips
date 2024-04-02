import matplotlib.pyplot as plt
import geopandas as gpd
import contextily as ctx
import osmnx as ox
from shapely.geometry import Point


def plot_stations_on_map1(G, stations):
    # Creating a list of dictionaries for each station with id, lat, lon, and nb_transfer
    data = [{
        'id': station_id,
        'lat': station.location.lat,
        'lon': station.location.lon,
        'nb_transfer': station.current_task.nb_transfer if station.current_task else 'N/A'
    } for station_id, station in stations.items()]

    # Create a list of Point geometries from the latitudes and longitudes
    geometry = [Point(lon, lat) for lon, lat in
                zip([station['lon'] for station in data], [station['lat'] for station in data])]

    # Create the GeoDataFrame directly with the 'geometry' column
    gdf = gpd.GeoDataFrame(data, geometry=geometry, crs='EPSG:4326')

    # Convert the GeoDataFrame to Web Mercator for contextily compatibility
    gdf_mercator = gdf.to_crs(epsg=3857)

    # Plotting
    fig, ax = plt.subplots(figsize=(15, 10))
    gdf_mercator.plot(ax=ax, color='red', markersize=70, alpha=0.6)  # Adjust markersize as needed

    # Add a basemap using OpenStreetMap as the basemap source
    ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)

    # Iterate over each row in the transformed GeoDataFrame to add text labels for nb_transfer
    for idx, row in gdf_mercator.iterrows():
        ax.text(row.geometry.x, row.geometry.y, str(row['nb_transfer']), color="black", ha="center", va="center",
                fontsize=6,  # Smaller font size
      #          fontweight='bold',  # Bold text
                transform=ax.transData)

    ax.set_axis_off()
    plt.show()

def plot_stations_on_map2(G):
    gdf = gpd.GeoDataFrame(
        [(data['Node']['id'], data['Node']['coordinates'][1], data['Node']['coordinates'][0]) for _, data in G.nodes(data=True)],
        columns=['id', 'lat', 'lon']
    )

    gdf['geometry'] = gpd.points_from_xy(gdf.lon, gdf.lat, crs='EPSG:4326')

    # Convert the GeoDataFrame to Web Mercator for contextily compatibility
    gdf_mercator = gdf.to_crs(epsg=3857)

    # Plotting
    fig, ax = plt.subplots(figsize=(15, 10))
    gdf_mercator.plot(ax=ax, color='red', markersize=30, alpha=0.6)

    # Add a basemap. Using OpenStreetMap as the basemap source.
    ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
    ax.set_axis_off()

    plt.show()


def plot_stations_on_map(G):
    # Define the location for the map (Montreal)
    place_name = "Montreal, Quebec, Canada"

    # Retrieve the street network for Montreal
    graph = ox.graph_from_place(place_name, network_type="drive")
    fig, ax = ox.plot_graph(graph, show=False, close=False, edge_color='gray', node_size=0)

    # Create a GeoDataFrame from the graph G
    gdf_stations = gpd.GeoDataFrame(
        [(data['Node']['id'], data['Node']['coordinates'][1], data['Node']['coordinates'][0]) for _, data in
         G.nodes(data=True)],
        columns=['id', 'lat', 'lon']
    )

    # Convert latitude and longitude to geometries
    gdf_stations['geometry'] = gpd.points_from_xy(gdf_stations.lon, gdf_stations.lat)

    # Plot the stations
    gdf_stations.plot(ax=ax, marker='o', color='red', markersize=30, alpha=0.6)

    # Remove the axis
    ax.set_axis_off()

    plt.show()
