import os

import pandas as pd


import constants
from Network.network import District_Network
from Simulation.create_vehicles import create_vehicles_files_proportional, create_vehicles_files_uniform
from Simulation.dataset import Dataset
from Simulation.vehicle import Vehicle
import constants as c
import Simulation.utilities as uf



def main_instance():
    create_vehicles = True
    folder_name = "Instances_16h-7"
    intance_dir = c.DATA_DIR + "/"+ folder_name + "/"

    """CREATE NETWORK"""
    manhattan_districts = District_Network()
    manhattan_districts.read_network_data(manhattan_geo_file='taxi_zones', stops_geojson='virtual_stops_latlon',
                                          edge_matrix_file='edge_matrix')

    vehicle_obj = Vehicle(len(manhattan_districts.districts))
    """CREATE DATASETS"""
    tests_data = []
    for files in uf.create_file_names():

        day_dataset = Dataset(files)
        average_requests = day_dataset.prepare_dataset(intance_dir, network=manhattan_districts, start_hr=7, end_hr=22,
                                                       start_min=0, end_min=59, capacity=4, visualize=True, remove_same=True)

        tests_data.append(['R' + str(day_dataset.nb_requests), day_dataset.file_name, day_dataset.nb_requests,
                           day_dataset.nb_customers, average_requests])

        vehicle_obj.update_nb_trip_per_district(network=manhattan_districts, dataset=day_dataset)

    """define instance names file"""
    tests_data = pd.DataFrame(tests_data, columns=['Instance', 'Dates', 'No. requests', 'No. customers', 'average'])
    tests_data.to_csv(intance_dir + 'data.csv', index=False)

    vehicle_obj.avg_trip_per_district()

    """CREATE VEHICLES"""
    if create_vehicles:
        create_vehicles_files_proportional(network=manhattan_districts, initial_vehicle=vehicle_obj, parent_dir=intance_dir)
        create_vehicles_files_uniform(network=manhattan_districts, initial_vehicle=vehicle_obj, parent_dir=intance_dir)



if __name__ == '__main__':
    main_instance()