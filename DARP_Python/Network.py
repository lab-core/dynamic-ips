from District import *
import json
import pandas as pd
import constants as c
import visualize as vf


class Network(object):
    def __init__(self, districts=None, outbound_cells=None, durations=None, cell_to_district=None,
                 cell_to_latitude=None, cell_to_longitude=None, locations=None, outbound_replace=None):
        if outbound_cells is None:
            outbound_cells = []
        if districts is None:
            districts = []
        if durations is None:
            durations = []
        if cell_to_district is None:
            cell_to_district = {}
        if cell_to_latitude is None:
            cell_to_latitude = {}
        if cell_to_longitude is None:
            cell_to_longitude = {}
        if outbound_replace is None:
            outbound_replace = {}
        if locations is None:
            locations = np.empty(shape=[0, 3])
        self.districts = districts
        self.outbound_cells = outbound_cells
        self.durations = durations
        self.cell_to_district = cell_to_district
        self.cell_to_latitude = cell_to_latitude
        self.cell_to_longitude = cell_to_longitude
        self.locations = locations
        self.outbound_replace = outbound_replace
        self.inbound_districts = []
        self.limited_districts = []


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
            self.cell_to_latitude[value] = float(key.split(",")[0])
            self.cell_to_longitude[value] = float(key.split(",")[1])
        self.locations = np.array(cells, dtype='double')

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
            if item.cartodb_id in [50, 247, 257, 142]:
                self.outbound_cells.extend(item.cells)
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
                          edge_time_matrix_file=None, stop_matrix_file=None, make_plot=False):
        self.read_district_data(manhattan_geo_file)
        if location_to_cell_file is not None:
            self.update_cells(location_to_cell_file)
        if edge_time_matrix_file is not None and stop_matrix_file is not None:
            self.read_duration_matrix(edge_time_matrix_file, stop_matrix_file)
        """determining outbound points replacements"""
        filtered_df = self.durations[self.durations['startID'].isin(self.outbound_cells[:,0])]
        filtered_data = filtered_df[~filtered_df['EndID'].isin(self.outbound_cells[:,0])]
        # Group the DataFrame by 'group' column and find the minimum value in 'startID' column
        min_df = filtered_data.groupby('startID')['duration'].min()
        # Merge the original DataFrame with the minimum values DataFrame on the 'startID' column
        df_result = pd.merge(filtered_data, min_df, on='startID')
        # Select only the rows where 'duration' is equal to the minimum value for each group
        df_result = df_result[df_result['duration_x'] == df_result['duration_y']]
        # Drop duplicates in 'startID' column and keep the first row
        df_result = df_result.drop_duplicates(subset='startID', keep='first')
        df_result = df_result.drop('duration_y', axis=1)
        df_result = df_result.rename(columns={'duration_x': 'duration'})
        for idx, row in df_result.iterrows():
            self.outbound_replace[row['startID']] = row['EndID']
        if make_plot:
            vf.plot_districts(district_network=self, print_id=True, add_legend=True,
                              file_name="districts")
            vf.plot_map_cells(district_network=self, print_id=True, pause=False, file_name="Map_Cells")

        """SELECT LIMITED DISTRICTS"""
        self.limited_districts = []
        for ID in c.LIMITED_DISTRICTS:
            for item in self.districts:
                if item.cartodb_id == ID:
                    self.limited_districts.append(item)
                    break

        """SELECT INBOUND DISTRICTS"""
        self.inbound_districts = []
        for ID in c.INBOUND_DISTRICTS:
            for item in self.districts:
                if item.cartodb_id == ID:
                    self.inbound_districts.append(item)
                    break
