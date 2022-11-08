from District import *
import json
import pandas as pd
import constants as c


class Network(object):
    def __init__(self, districts=None, outbound_cells=None, durations=None, cell_to_district=None):
        if outbound_cells is None:
            outbound_cells = []
        if districts is None:
            districts = []
        if durations is None:
            durations = []
        if cell_to_district is None:
            cell_to_district = {}
        self.districts = districts
        self.outbound_cells = outbound_cells
        self.durations = durations
        self.cell_to_district = cell_to_district

    def __str__(self):
        class_string = str(self.__class__) + ": {"
        for attribute, value in self.__dict__.items():
            class_string += str(attribute) + ": " + str(value) + ",\n"
        class_string += "}"
        return class_string

    def read_district_data(self, manhattan_geo_file):
        """ read districts from a file """
        file = open(c.NETWORK_DIR + manhattan_geo_file + ".json")
        manhattan_geo = json.load(file)
        file.close()
        for item in range(len(manhattan_geo['features'])):
            district_id = manhattan_geo['features'][item]['properties']['cartodb_id']
            district_name = manhattan_geo['features'][item]['properties']['name']
            district_coordinates = np.array(manhattan_geo['features'][item]['geometry']['coordinates'][0][0])
            np.array(district_coordinates)
            district_coordinates[:, [1, 0]] = district_coordinates[:, [0, 1]]

            district = District(district_id, district_name, district_coordinates)
            self.districts.append(district)

    """ this function get a location coordinate and finds its corresponding district index """

    def get_district_id(self, lat, long):
        for index, item in enumerate(self.districts):
            if item.point_inside_district(lat, long):
                return index

    def update_cells(self, location_to_cell_file):
        self.outbound_cells.clear()

        """ read districts from a file """
        file = open(c.NETWORK_DIR + location_to_cell_file + ".json")
        location_to_cell = json.load(file)
        file.close()

        cells = []
        self.cell_to_district = {}
        location_to_cell = dict(sorted(location_to_cell.items(), key=lambda item: item[1]))
        for index, (key, value) in enumerate(location_to_cell.items()):
            cells.append([value, float(key.split(",")[0]), float(key.split(",")[1])])
        for cell in cells:
            cell_added = False
            for item in self.districts:
                if item.point_inside_district(cell[1], cell[2]):
                    item.cells.append(cell)
                    cell_added = True
                    self.cell_to_district[cell[0]] = item.cartodb_id
                    break
            if cell_added is False:
                self.outbound_cells.append(cell)
                self.cell_to_district[cell[0]] = c.OUT_BOUND
        for item in self.districts:
            item.cells = np.array(item.cells)
        self.outbound_cells = np.array(self.outbound_cells)

    def read_duration_matrix(self, edge_time_matrix_file, stop_matrix_file):
        # read data
        file = open(c.NETWORK_DIR + edge_time_matrix_file + ".json")
        df_edge_time_matrix = json.load(file)
        file.close()

        file = open(c.NETWORK_DIR + stop_matrix_file + ".json")
        df_stop_matrix = json.load(file)
        file.close()
        """ covert data to matrix """
        duration_data = []
        k = 0
        for i in range(len(df_stop_matrix)):
            for j in range(len(df_stop_matrix)):
                duration_data.append([i, j, df_edge_time_matrix[k]])
                k = k + 1
        self.durations = pd.DataFrame(duration_data, columns=['startID', 'EndID', 'duration'])

    def save_duration_txt(self, duration_filename):
        """ save data file """
        duration_file = c.NETWORK_DIR + duration_filename + ".txt"
        df_columns = self.durations.columns.tolist()
        file = open(duration_file, "w")
        file.write("COLUMNS\n\n")

        for col in df_columns:
            file.write(col)
            file.write("\n")
        file.write("\nDURATION_INFO\n")
        df_as_string = self.durations.to_string(header=False, index=False)
        file.write(df_as_string)
        file.close()

    def read_network_data(self, manhattan_geo_file, location_to_cell_file=None,
                          edge_time_matrix_file=None, stop_matrix_file=None):
        self.read_district_data(manhattan_geo_file)
        if location_to_cell_file is not None:
            self.update_cells(location_to_cell_file)
        if edge_time_matrix_file is not None and stop_matrix_file is not None:
            self.read_duration_matrix(edge_time_matrix_file, stop_matrix_file)

