import constants as c
import utilities as uf
import visualize as vf
from Dataset import *


def main(name):
    manhattan_districts = Network()
    manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                          edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix')
    vf.plot_districts(district_network=manhattan_districts,print_id=True,add_legend=True, file_name="districts")
    for files in uf.create_file_names():
        day_dataset = Dataset(files)
        day_dataset.prepare_dataset(network=manhattan_districts, capacity=4)
        vf.show_request_per_hr(day_dataset.dataset, day_dataset.origin, save_image=True)
        day_dataset.limit_time_dataset(start_hr=8, end_hr=19)
        day_dataset.save_dataset()
        vf.plot_districts_by_nb_trips(trip_per_district=day_dataset.nb_trip_per_district,
                                      district_network=manhattan_districts,print_id=True,file_name=day_dataset.file_name)
        day_dataset.save_instance()
        #vf.plot_map_cells(manhattan_districts, True, pause=True, file_name=day_dataset.file_name)

if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
