import constants as c
import utilities as uf
import visualize as vf
from Dataset import *
from Vehicle import *
import csv


def main(name):
    folder_name = "SMALL"
    """CREATE NETWORK"""
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix',
                                          make_plot=False)
    manhattan_districts.save_duration_txt(folder_name, "edge_time_matrix")
    manhattan_districts.save_graph_to_json(folder_name, "graph")

    vehicle_obj = Vehicle(len(manhattan_districts.districts))
    """CREATE DATASETS"""
    tests_data = []
    for files in uf.create_file_names():
        day_dataset = Dataset(files)
        day_dataset.prepare_dataset(folder_name, network=manhattan_districts, start_hr=8, end_hr=9, start_min=0, end_min=0,
                                    capacity=4, restrict=c.SMALL_REGION, visualize=True, remove_same=True)
        tests_data.append(['R' + str(day_dataset.nb_requests), day_dataset.file_name, day_dataset.nb_requests,
                           day_dataset.nb_customers])
        vehicle_obj.update_nb_trip_per_district(network=manhattan_districts, dataset=day_dataset)

    """define instance names file"""
    tests_data = pd.DataFrame(tests_data, columns=['Instance', 'Dates', 'No. requests', 'No. customers'])
    tests_data.to_csv(c.DAYS_DIR + "Instances/" + 'data.csv', index=False)

    vehicle_obj.avg_trip_per_district()

    """CREATE VEHICLES"""
    for nb_vehicles in c.NB_VEHICLES_SMALL:
        start_seconds, end_seconds = uf.calculate_time_from_origin_sec(start_hr=8, end_hr=9, start_min=0, end_min=0)
        uf.create_vehicles_file_small(network=manhattan_districts, initial_vehicle=vehicle_obj,parent_dir=folder_name,
                                    nb_vehicle=nb_vehicles, start_time=start_seconds)


if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
