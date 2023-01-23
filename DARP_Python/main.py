import constants as c
import utilities as uf
import visualize as vf
from Dataset import *
from Vehicle import *


def main(name):
    """CREATE NETWORK"""
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix',
                                          make_plot=True)
    vehicle_obj = Vehicle(len(manhattan_districts.districts))
    """CREATE DATASETS"""
    for files in uf.create_file_names():
        day_dataset = Dataset(files)
        day_dataset.prepare_dataset(network=manhattan_districts)
        vf.show_dataset_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
        day_dataset.limit_time_dataset(start_hr=7, end_hr=8, start_min=0, end_min=0)
        day_dataset.visualize_dataset(network=manhattan_districts)
        day_dataset.split_requests(capacity=4)
        day_dataset.save_dataset()
        day_dataset.save_instance()
        vehicle_obj.update_nb_trip_per_district(network=manhattan_districts, dataset=day_dataset)
    vehicle_obj.avg_trip_per_district()

    """SELECT DISTRICTS"""
    selected_districts = []
    for ID in c.RESTRICTED_DISTRICTS:
        for item in manhattan_districts.districts:
            if item.cartodb_id == ID:
                selected_districts.append(item)
                break

    """CREATE VEHICLES"""
    uf.create_vehicles_files(network=manhattan_districts, initial_vehicle=vehicle_obj)
    uf.create_vehicles_from_files(network=manhattan_districts)
    uf.create_vehicles_from_files(network=manhattan_districts, selected_districts=selected_districts)


#    vf.plot_unused_vehicle(manhattan_districts, result_file="STATIC_GREEDY_20230121-1226",
#                           instance_name="20150706_07-60m", instance_folder="Instances-60", nb_vehicles=1500)


if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
