import constants as c
import utilities as uf
import visualize as vf
from Dataset import *
from Vehicle import *
import csv


def main(name):
    # Replace these with your folder path and desired output file name
 #   uf.merge_csv_files(instance_folder="Instances-300-1")

    """CREATE NETWORK"""
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix',
                                          make_plot=True)
    vf.plot_map_cells(manhattan_districts, print_id=True, pause=False, file_name="Map_Cells1")
    vehicle_obj = Vehicle(len(manhattan_districts.districts))
    vf.plot_sink_vehicle(manhattan_districts, result_file="ANYTIME_GREEDY_20230831-0410", instance_name="20160129_17-300m_1",
                           instance_folder="Instances-300-1", nb_vehicles=1800, method="A_GREEDY")

#    uf.create_vehicles_files_random_remove(network=manhattan_districts, file_name='vehicles_2000_4.csv')
    """CREATE DATASETS"""

    """SELECT DISTRICTS"""
    limited_districts = []
    for ID in c.LIMITED_DISTRICTS:
        for item in manhattan_districts.districts:
            if item.cartodb_id == ID:
                limited_districts.append(item)
                break

    tests_data = []

    for files in uf.create_file_names():
        day_dataset = Dataset(files)
        day_dataset.prepare_dataset(network=manhattan_districts)
        vf.show_dataset_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
        day_dataset.limit_time_dataset(start_hr=17, end_hr=22, start_min=0, end_min=0)
        day_dataset.remove_same_pick_drop()
        day_dataset.restrict_to_district(c.INBOUND_DISTRICTS)
        day_dataset.split_requests(capacity=4)
        print("\nThe number of trips after time limit:", day_dataset.nb_requests)
        print("The number of customers after time limit:", day_dataset.nb_customers)
        day_dataset.calculate_trip_per_district(network=manhattan_districts)
        day_dataset.visualize_dataset(network=manhattan_districts)
        day_dataset.save_dataset()
        day_dataset.save_instance()
        vehicle_obj.update_nb_trip_per_district(network=manhattan_districts, dataset=day_dataset)
        tests_data.append(['R'+ str(day_dataset.nb_requests), day_dataset.file_name, day_dataset.nb_requests,
                day_dataset.nb_customers])

    vehicle_obj.avg_trip_per_district()

    """define instance names file"""
    tests_data = pd.DataFrame(tests_data, columns=['Instance', 'Dates', 'No. requests', 'No. customers'])
    tests_data.to_csv(c.DAYS_DIR + "Instances/"+'data.csv', index=False)

    """SELECT INBOUND DISTRICTS"""
    inbound_districts = []
    for ID in c.INBOUND_DISTRICTS:
        for item in manhattan_districts.districts:
            if item.cartodb_id == ID:
                inbound_districts.append(item)
                break

    """CREATE VEHICLES"""
    uf.create_vehicles_files(network=manhattan_districts, initial_vehicle=vehicle_obj)
    uf.create_vehicles_from_files(network=manhattan_districts, replace=True)
    uf.create_vehicles_from_files(network=manhattan_districts, selected_districts=inbound_districts)


#    vf.plot_unused_vehicle(manhattan_districts, result_file="STATIC_GREEDY_20230217-0809",
#                           instance_name="20150706_07-60m", instance_folder="Instances-60", nb_vehicles=1500)


if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
