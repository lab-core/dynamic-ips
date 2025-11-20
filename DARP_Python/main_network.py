from Network.create_network import create_virtual_stops, create_taxi_zone_shapefile, \
    create_riley_virtual_stops_from_real_points
from Network.get_distance_data import process_virtual_stops
from Network.post_process_network import postprocess_json_matrix, convert_json_matrix_to_txt
from Visualization.visualize_network import plot_virtual_stops
from Network.network import District_Network

def main_create_stops():
    create_taxi_zone_shapefile()
    create_virtual_stops()
    create_riley_virtual_stops_from_real_points()

    plot_virtual_stops(output_filename="virtual_stops_with_zoom.png", input_stop_file="virtual_stops.geojson",
                       zoom_bounds=(585000, 588000, 4513000, 4516000))

    plot_virtual_stops(output_filename="rely_virtual_stops_with_zoom.png",
                       input_stop_file="riley_virtual_stops.geojson",
                       zoom_bounds=(585000, 588000, 4513000, 4516000), stop_color='maroon')
    """
    riley_input_path = "riley_virtual_stops_latlon.geojson"
    riley_output_name = "riley_edge_matrix.json"
    
    # Process the stops
    riley_stop_ids, riley_dist_matrix, riley_dur_matrix = process_virtual_stops(riley_input_path, riley_output_name)
    convert_json_matrix_to_txt("riley_edge_matrix.json", "riley_edge_time_matrix")
    """

    input_path = "virtual_stops_latlon.geojson"
    output_name = "edge_matrix.json"

    # Process the stops
    stop_ids, dist_matrix, dur_matrix = process_virtual_stops(input_path, output_name)
    convert_json_matrix_to_txt("edge_matrix.json", "edge_time_matrix")



def main_network():
    """CREATE NETWORK"""
    manhattan_districts = District_Network()
    manhattan_districts.read_network_data(manhattan_geo_file='taxi_zones', stops_geojson = 'virtual_stops_latlon',
                                          edge_matrix_file = 'edge_matrix')

if __name__ == '__main__':
    main_create_stops()
    postprocess_json_matrix("edge_matrix.json")
