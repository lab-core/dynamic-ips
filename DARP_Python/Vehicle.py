from District import *
import json
import pandas as pd
import constants as c
import math
import os
import visualize as vf
import utilities as uf
import random


class Vehicle(object):
    def __init__(self, nb_districts, vehicle_data=None, nb_vehicle_per_district=None, file_name=None, nb_vehicles=None,
                 capacity=None, nb_trip_per_district=None):
        if vehicle_data is None:
            vehicle_data = []
        if file_name is None:
            file_name = "vehicles_2000_4"
        if nb_vehicles is None:
            nb_vehicles = 2000
        if capacity is None:
            capacity = 4
        if nb_trip_per_district is None:
            nb_trip_per_district = [0] * nb_districts
        if nb_vehicle_per_district is None:
            nb_vehicle_per_district = []
        self.nb_trip_per_district = nb_trip_per_district
        self.nb_vehicle_per_district = nb_vehicle_per_district
        self.vehicle_data = vehicle_data
        self.nb_vehicles = nb_vehicles
        self.capacity = capacity
        self.file_name = file_name

    def create_vehicle_data_from_file(self, network, selected_districts=None, replace=False):
        """ read data """
        f = open(c.VEHICLES_DIR + self.file_name + ".json")
        df_vehicles = json.load(f)
        f.close()

        self.nb_vehicles = len(df_vehicles)

        if selected_districts is not None:
            source_ids = []
            for item in selected_districts:
                for cell in item.cells:
                    source_ids.append(int(cell[0]))
        # covert data to matrix
        vehicle_data = []
        for i in range(self.nb_vehicles):
            source_id = df_vehicles[i]['start_stop_id']
            if selected_districts is not None:
                source_id = source_ids[i % len(source_ids)]
            if replace:
                if int(source_id) in network.outbound_replace:
                    source_id = network.outbound_replace[int(source_id)]
            zone_id = network.cell_to_district[int(source_id)]
            vehicle_data.append(
                [i, df_vehicles[i]['capacity'], df_vehicles[i]['start_time'], 90000, source_id, source_id, zone_id])

        self.vehicle_data = pd.DataFrame(vehicle_data,
                                         columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time', 'depart_ID',
                                                  'sink_ID', 'zone_ID'])

    def create_vehicle_data_from_districts(self, network, start_time):
        vehicle_data = []
        vehicle_id = 0
        for count, item in enumerate(network.districts):
            source_ids = []
            for cell in item.cells:
                source_ids.append(int(cell[0]))
            source_ids = sorted(source_ids)
            select_source_ids = random.choices(source_ids, k=self.nb_vehicle_per_district[count])
            for i in range(self.nb_vehicle_per_district[count]):
                source_id = select_source_ids[i];
  #              source_id = source_ids[i % len(source_ids)]
                zone_id = network.cell_to_district[int(source_id)]
                vehicle_data.append([vehicle_id, self.capacity, start_time, 90000, source_id, source_id, zone_id])
                vehicle_id = vehicle_id + 1
        self.vehicle_data = pd.DataFrame(vehicle_data, columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time',
                                                                'depart_ID', 'sink_ID', 'zone_ID'])

    def create_vehicle_data_from_districts_uni(self, network):
        vehicle_data = []
        vehicle_id = 0
        for count, item in enumerate(network.districts):
            source_ids = []
            for cell in item.cells:
                source_ids.append(int(cell[0]))
            source_ids = sorted(source_ids)
            if (self.nb_vehicle_per_district[count] < len(source_ids)):
                select_source_ids = random.sample(source_ids, k=self.nb_vehicle_per_district[count])
            else:
                if (len(source_ids) != 0):
                    times = self.nb_vehicle_per_district[count] // len(source_ids)
                    remaining = self.nb_vehicle_per_district[count] % len(source_ids)
                    select_source_ids = random.choices(source_ids, k=remaining)
                    for i in range(times):
                        select_source_ids = select_source_ids + source_ids
                else:
                    select_source_ids = random.choices(source_ids, k=self.nb_vehicle_per_district[count])
            for i in range(self.nb_vehicle_per_district[count]):
                source_id = select_source_ids[i];
                #              source_id = source_ids[i % len(source_ids)]
                zone_id = network.cell_to_district[int(source_id)]
                vehicle_data.append([vehicle_id, self.capacity, 0, 90000, source_id, source_id, zone_id])
                vehicle_id = vehicle_id + 1
        self.vehicle_data = pd.DataFrame(vehicle_data, columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time',
                                                                'depart_ID', 'sink_ID', 'zone_ID'])

    def update_nb_trip_per_district(self, network, dataset):
        dataset.calculate_trip_per_district(network)
        self.nb_trip_per_district = [self.nb_trip_per_district[i] + dataset.nb_trip_per_district[i] for i in
                                     range(len(self.nb_trip_per_district))]
        self.nb_trip_per_district[1] = 0

    def avg_trip_per_district(self):
        self.nb_trip_per_district = [math.floor(self.nb_trip_per_district[i] / 24) for i in
                                     range(len(self.nb_trip_per_district))]

    def calculate_vehicle_per_district(self):
        self.nb_vehicle_per_district.clear()
        total = 0
        for i in range(len(self.nb_trip_per_district)):
            self.nb_vehicle_per_district.append(
                round((self.nb_vehicles * self.nb_trip_per_district[i]) / sum(self.nb_trip_per_district)))
            total = total + self.nb_vehicle_per_district[i]
        if total != self.nb_vehicles:
            self.nb_vehicle_per_district[0] = self.nb_vehicle_per_district[0] + self.nb_vehicles - total

    def save_vehicle(self, folder_name, parent_dir):
        # save data file
        folder_name = c.DAYS_DIR + parent_dir + "/" + folder_name
        if not os.path.exists(folder_name):
            os.mkdir(folder_name)

        file_to_save = folder_name + "/" + self.file_name + ".txt"
        csv_file = folder_name + "/" + self.file_name + ".csv"
        df_columns = self.vehicle_data.columns.tolist()
        file = open(file_to_save, "w")
        file.write("COLUMNS\n\n")

        for col in df_columns:
            file.write(col)
            file.write("\n")
        file.write("\nVEHICLES_INFO\n")
        df_as_string = self.vehicle_data.to_string(header=False, index=False)
        df_csv = self.vehicle_data
        df_csv.drop(columns=['end_Time', 'sink_ID', 'zone_ID'])
        df_csv.rename(columns={'depart_Time': 'start_time', 'depart_ID': 'start_stop',
                               'vehicle_ID': 'vehicle_id'}, inplace=True)
        df_csv = df_csv[['vehicle_id','start_time','start_stop','capacity']]

        df_csv.to_csv(csv_file, index=False, sep=';')
        file.write(df_as_string)
        file.close()

    def plot_map_vehicle_cells(self, district_network, print_id, folder_name):
        fig, ax = vf.plot_districts(district_network, print_id)
        points = np.array(self.vehicle_data["depart_ID"].astype('int'))

        vehicle_cells = np.array(
            [district_network.locations[index] for index in (np.array(self.vehicle_data["depart_ID"].astype('int')))])
        vehicle_cells = pd.DataFrame(vehicle_cells, columns=['cell_ID', 'latitude', 'longitude'])
        vehicle_points = vehicle_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
            name='cell_size')
        y = np.array(vehicle_points['latitude'])
        x = np.array(vehicle_points['longitude'])
        cell_size = 3 * np.array(vehicle_points['cell_size'])
        colors = np.random.randint(100, size=(len(x)))
        # plt.scatter(x, y, c=colors, s=cell_size, alpha=0.5, cmap='nipy_spectral')
        plt.scatter(x, y, c='red', s=cell_size, alpha=1)
#        ax.set_title(self.file_name, fontsize=13, fontweight='bold', y=1.0, pad=-14)

        parent_folder = c.DAYS_DIR + folder_name
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + self.file_name, dpi=300, bbox_inches="tight")
        plt.close(fig)
