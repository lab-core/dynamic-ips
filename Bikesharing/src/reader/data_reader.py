import os
from datetime import datetime

from src.simulator.resource import Vehicle
from src.simulator.station import Station
from src.simulator.parameters import Solver_Parameters, Parameters
from src.simulator.stop import Location, Stop
import json
import networkx as nx
import ast
from collections import OrderedDict
from src.simulator.task import Task, TaskStatus



class DataReader:
    """
    DataReader is responsible for reading and processing input data from files,
    including tasks, vehicle information, transportation network, and the parameters.

    Attributes
    ----------
    task_folder_path : str, optional
        Folder path to the folder containing the JSON file for the re-balancing tasks.
    vehicles_file_path : str, optional
        File path to the JSON file containing vehicle data.
    duration_file_path : str, optional
        File path to the JSON file containing the duration matrix.
    station_file_path : str, optional
        File path to the JSON file containing the station's data.
    parameter_file_path : str, optional
        File path to the JSON file containing the BIXI parameter's data.
    algorithm_parameter_file_path : str, optional
        File path to the JSON file containing the algorithm parameter's data.
    """

    def __init__(self, task_folder_path=None, vehicles_file_path=None, duration_file_path=None,
                 station_file_path=None, parameter_file_path=None, algorithm_parameter_file_path=None):
        self.task_folder_path = task_folder_path
        self.vehicles_file_path = vehicles_file_path
        self.duration_file_path = duration_file_path
        self.station_file_path = station_file_path
        self.parameter_file_path = parameter_file_path
        self.algorithm_parameter_file_path = algorithm_parameter_file_path

    def get_json_parameters(self):
        """
            Loads parameters from the JSON file specified in parameter_file_path.

            Returns
            -------
            params
                An instance of Parameters class initialized with data from the JSON file.
        """
        if self.parameter_file_path is None:
            raise ValueError("Parameters file path is not set.")

        with open(self.parameter_file_path, 'r') as file:
            data = json.load(file)

        params = Parameters(
            simulation_start=data["simulation_start"],
            num_vehicles=data["num_vehicles"],
            num_stations=data["num_stations"],
            time_per_bike=data["time_per_bike"],
            time_to_park=data["time_to_park"]
        )

        return params

    def get_json_solver_parameters(self):
        """
        Loads algorithm parameters from the JSON file specified in algorithm_parameter_file_path.

        Returns
        -------
        Parameters
            An instance of Solver_Parameters class initialized with data from the JSON file.
        """
        if self.algorithm_parameter_file_path is None:
            raise ValueError("Algorithm parameters file path is not set.")

        with open(self.algorithm_parameter_file_path, 'r') as file:
            data = json.load(file)

        params = Solver_Parameters(
            epoch_length=data.get("epoch_length", 30),
            nb_threads=data.get("nb_threads", 8),
            main_algorithm=data.get("main_algorithm", 0),
            nb_column=data.get("nb_column", 60),
            is_truncated=bool(data.get("is_truncated", 1)),
            max_label=data.get("max_label", 15),
            is_drop_pick_possible=bool(data.get("is_drop_pick_possible", 0)),
            labeling_strategy=data.get("labeling_strategy", 0),
            nb_pick=data.get("nb_pick", 2),
            sort_path=data.get("sort_path", 0),
            mip_gap=data.get("mip_gap", 0.001)
        )

        return params

    def get_json_stations(self):
        """
        Reads station data from a JSON file and converts it into Station objects.

        Returns
        -------
        stations: dict
            A dictionary of Station objects initialized with data from the JSON file, keyed by station_id.

        locations: dict
            A dictionary containing the coordinates for each station, keyed by location_id.

        G : networkx.Graph
            A graph with nodes representing station locations (using location IDs as node labels).
        """
        if self.station_file_path is None:
            raise ValueError("Station file path is not set.")

        with open(self.station_file_path, 'r') as file:
            stations_data = json.load(file)

        stations = {}
        locations = {}  # Initialize an empty dictionary for locations
        G = nx.Graph()  # Initialize an empty graph

        for key, data in stations_data.items():
            location_id = data["location_id"]
            location = Location(label=location_id, lon=data["longitude"], lat=data["latitude"])
            station = Station(
                station_id=data["station_id"],
                short_name=data["short_name"],
                name=data["name"],
                location=location,
                capacity=data["capacity"],
                sector=data["sector"]
            )
            # Populate the locations dictionary
            locations[location_id] = location

            # Prepare node attributes for graph G
            node_dict = {
                "id": location_id,
                "coordinates": [float(data["longitude"]), float(data["latitude"])],
            }
            stations[data["station_id"]] = station
            G.add_node(location_id, Node=node_dict)
        return stations, locations, G

    def get_json_duration(self):
        """
        Reads duration data from a JSON file where keys are location IDs.

        Returns
        -------
        dict
            A nested dictionary where the outer keys are source location IDs, inner keys
            are destination location IDs, and values are durations.
        """
        if self.duration_file_path is None:
            raise ValueError("Duration file path is not set.")

        with open(self.duration_file_path, 'r') as file:
            duration_data = json.load(file)

            # Transforming the JSON structure into the nested dictionary format
        durations = {
            int(outer_key): {int(inner_key): int(inner_value) for inner_key, inner_value in outer_value.items()}
            for outer_key, outer_value in duration_data.items()}

        return durations

    def get_duration(self):
        """
        Reads duration data from a file where keys are location IDs and evaluates it into a nested dictionary.

        Returns
        -------
        dict
            A nested dictionary where the outer keys are source location IDs, inner keys
            are destination location IDs, and values are durations.
        """
        if self.duration_file_path is None:
            raise ValueError("Duration file path is not set.")

        try:
            with open(self.duration_file_path, 'r') as file:
                duration_data_str = file.read()
                # Safely evaluate string to Python dictionary
                duration_data = ast.literal_eval(duration_data_str)
        except SyntaxError as e:
            raise ValueError(f"Error processing the duration data: {e}")

        # Transforming the dictionary structure into the nested dictionary format
        durations = {
            int(outer_key): {int(inner_key): int(inner_value) for inner_key, inner_value in outer_value.items()}
            for outer_key, outer_value in duration_data.items()}

        return durations

    def get_sorted_duration(self):
        """
        Reads duration data from a file, evaluates it into a nested dictionary, and sorts it.

        Returns
        -------
        OrderedDict
            A sorted nested dictionary where the outer keys are source location IDs, inner keys
            are destination location IDs, and values are durations.
        """
        if self.duration_file_path is None:
            raise ValueError("Duration file path is not set.")

        try:
            with open(self.duration_file_path, 'r') as file:
                duration_data_str = file.read()
                duration_data = ast.literal_eval(duration_data_str)
        except SyntaxError as e:
            raise ValueError(f"Error processing the duration data: {e}")

        # Sorting the outer dictionary
        sorted_duration_data = OrderedDict(sorted(duration_data.items(), key=lambda x: int(x[0])))

        # Sorting each nested dictionary
        for key in sorted_duration_data:
            sorted_duration_data[key] = OrderedDict(sorted(sorted_duration_data[key].items(), key=lambda x: int(x[0])))

        return sorted_duration_data

    def update_tasks(self, stations, current_time):
        closest_time = None
        closest_file = None

        # Format the current_date string to match the date part of the filenames
        current_date_str = current_time.strftime("%Y-%m-%d")
        base_time = current_time.replace(hour=0, minute=0, second=0, microsecond=0)

        # Filter filenames to only those matching the current date
        filtered_filenames = [filename for filename in os.listdir(self.task_folder_path)
                              if filename.startswith(f"cost_{current_date_str}")]

        # Iterate over filtered files in the given folder
        for filename in filtered_filenames:
            # Extract the datetime part of the filename and convert it into a datetime object
            file_time_str = filename.replace("cost_", "").replace("/", ":").replace(".json","")
            try:
                file_time = datetime.strptime(file_time_str, "%Y-%m-%d %H:%M:%S")
                # Check if this file's time is the closest smaller than the current_time
                if file_time < current_time and (closest_time is None or file_time > closest_time):
                    closest_time = file_time
                    closest_file = filename
            except ValueError as e:
                print(f"Error parsing date from {filename}: {e}")

        # If a closest file was found, read and return its content
        if closest_file is not None:
            with open(os.path.join(self.task_folder_path, closest_file), 'r') as file:
                task_data = json.load(file)

                for data in task_data:
                    station_id = data['stationID']
                    if station_id in stations:
                        # Update the current_task of the station with new data
                        transfer = data['transfer']
                        if transfer > 0:
                            relocate_cost = (-1) * data['minus_relocat_cost']
                            # Additional attributes can be extracted and set here as needed
                            # Assuming stations[station_id] has a method to update its current task
                            # and Task class is properly defined to accept all necessary parameters.
                            if stations[station_id].current_task.task_status == TaskStatus.INITIALIZED:
                                stations[station_id].current_task.release(transfer, relocate_cost, closest_time,base_time)
                            else:
                                stations[station_id].current_task.update(transfer, relocate_cost, closest_time, base_time)

    def get_json_vehicles(self, locations):
        """
        Reads the resources JSON file, and creates Vehicle objects for each resource entry.

        Returns
        -------
        dict
            A dictionary of Vehicle objects created from the resources data, keyed by vehicle IDs.
        """
        with open(self.vehicles_file_path, 'r') as file:
            resources_data = json.load(file)

        vehicles = {}  # Initialize an empty dictionary instead of a list
        for resource in resources_data:
            # Assuming 'depart_stop' can be a default or null value
            vehicle = Vehicle(vehicle_id=resource["ID"],
                              start_time=resource["StartTime"],
                              end_time=resource["EndTime"],
                              depart_stop=Stop(arrival_time=resource["StartTime"],depart_time=float('inf'),
                                               nb_bike_to_pick=0, nb_bike_to_drop=0,
                                               location=locations[resource["Location"]]),
                              capacity=resource["Capacity"],
                              bike_load=resource.get("Inventory", 0))  # Defaulting to 0 if not available
            # Add the vehicle to the dictionary with its ID as the key
            vehicles[vehicle.vehicle_id] = vehicle

        return vehicles
