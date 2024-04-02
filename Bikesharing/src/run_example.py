import logging  # Required to modify the log level
import os
from datetime import datetime

from src.reader.data_reader import DataReader
from src.reader.visualization import plot_stations_on_map, plot_stations_on_map1

if __name__ == '__main__':

    # To modify the log level (at INFO, by default)
    logging.getLogger().setLevel(logging.DEBUG)

    # Read input data from files
    print("Current Working Directory:", os.getcwd())
    parameters_path = "../data/parameters.json"
    solver_parameters_path = "../data/solver_parameters.json"
    stations_path = "../data/stations.json"
    duration_path = "../data/time_matrix.json"
    task_path = "../data/tasks_data"
    resource_path = "../data/resources.json"

    data_reader = DataReader(duration_file_path=duration_path, station_file_path=stations_path,
                             parameter_file_path=parameters_path, algorithm_parameter_file_path=solver_parameters_path,
                             task_folder_path=task_path, vehicles_file_path=resource_path)

    duration_matrix = data_reader.get_sorted_duration()
    stations, locations, network = data_reader.get_json_stations()
    parameters = data_reader.get_json_parameters()
    parameters.num_stations = len(stations)
    solver_parameters = data_reader.get_json_solver_parameters()
    resources = data_reader.get_json_vehicles(locations)
    # Example usage

    current_time = datetime(2024, 3, 20, 00, 11, 0)  # Example current time as datetime object
    data_reader.update_tasks(stations, current_time)


    plot_stations_on_map1(network, stations)





