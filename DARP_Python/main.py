import constants as c
import utilities as uf
import visualize as vf
from Dataset import *
from Vehicle import *
import csv


def main(name):
    create_dataset = True

    inst_folder = "Instances-300_1-f1"
    start = 64800
    algorithm_list = ["A_MP_MIP", "A_MP_CG", "A_MP_ISUD", "A_GREEDY"]
    algorithm_list2 = ["S_MP_MIP", "S_MP_CG", "S_MP_ISUD"]

    """CREATE NETWORK"""
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix',
                                          make_plot=True)
    if inst_folder != "":
        uf.merge_change_solutions(instance_folder=inst_folder, algorithms=algorithm_list2,)
        uf.merge_csv_files(instance_folder=inst_folder)
        uf.merge_requests(instance_folder=inst_folder, algorithms=algorithm_list, start_time=start)

        vf.plot_vehicle_states(manhattan_districts, instance_folder=inst_folder, nb_vehicles=1800)
        vf.create_plots(instance_folder=inst_folder, algorithms=algorithm_list, start_time=start, save_image=True,
                        os_type="osx", pause=False, Title=True)

    if create_dataset:
        vehicle_obj = Vehicle(len(manhattan_districts.districts))
        """CREATE DATASETS"""
        tests_data = []
        for files in uf.create_file_names():
            day_dataset = Dataset(files)
            day_dataset.prepare_dataset(network=manhattan_districts, start_hr=17, end_hr=22, start_min=0, end_min=0,
                                        capacity=4, restrict=c.INBOUND_DISTRICTS, visualize=True, remove_same=True)
            tests_data.append(['R' + str(day_dataset.nb_requests), day_dataset.file_name, day_dataset.nb_requests,
                               day_dataset.nb_customers])
            vehicle_obj.update_nb_trip_per_district(network=manhattan_districts, dataset=day_dataset)

        """define instance names file"""
        tests_data = pd.DataFrame(tests_data, columns=['Instance', 'Dates', 'No. requests', 'No. customers'])
        tests_data.to_csv(c.DAYS_DIR + "Instances/" + 'data.csv', index=False)

        vehicle_obj.avg_trip_per_district()

        """CREATE VEHICLES"""
        uf.create_vehicles_files(network=manhattan_districts, initial_vehicle=vehicle_obj)
        uf.create_vehicles_from_files(network=manhattan_districts, replace=True)
        uf.create_vehicles_from_files(network=manhattan_districts,
                                      selected_districts=manhattan_districts.inbound_districts)


if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
