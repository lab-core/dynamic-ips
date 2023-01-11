import constants as c
import utilities as uf
import visualize as vf
from pathlib import Path
import glob
from Dataset import *


def main(name):
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix')
    selected_districts = []
    for id in c.LIMITED_DISTRICTS:
        for item in manhattan_districts.districts:
            if item.cartodb_id == id:
                selected_districts.append(item)
                break
    for file in glob.glob(c.VEHICLES_DIR + "*.json"):
        uf.create_resticted_vehicle_dataset(Path(file).stem, selected_districts, manhattan_districts.cell_to_district)
        uf.create_vehicle_dataset_with_zone(Path(file).stem, manhattan_districts)
 #   vf.plot_districts(district_network=manhattan_districts,print_id=True,add_legend=True, file_name="districts")
    for files in uf.create_file_names():
        day_dataset = Dataset(files)
        day_dataset.prepare_dataset(network=manhattan_districts)
        day_dataset.restrict_to_district(selected_districts)

 #       vf.show_dataset_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
#        vf.show_request_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
#        vf.show_customer_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
        day_dataset.limit_time_dataset(start_hr=7, end_hr=8, start_min=0, end_min=0)
        day_dataset.split_requests(capacity = 4)
#        day_dataset.calculate_trip_per_district(network=manhattan_districts)
#        day_dataset.calculate_vehicle_per_district(network=manhattan_districts)

#        for file in glob.glob(c.VEHICLES_DIR + "*.json"):
#            uf.create_vehicle_dataset_zone(Path(file).stem, districts=manhattan_districts.districts, nb_per_zone=day_dataset.nb_vehicle_per_district)

        day_dataset.save_dataset()
#        vf.plot_districts_by_nb_trips(trip_per_district=day_dataset.nb_trip_per_district,
#                                      district_network=manhattan_districts,print_id=True,file_name=day_dataset.file_name)
        day_dataset.save_instance()
        #vf.plot_map_cells(manhattan_districts, True, pause=True, file_name=day_dataset.file_name)

if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
