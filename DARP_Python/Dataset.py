import math
from datetime import datetime as dt
import os
import visualize as vf
from Network import *
import datetime as dtime
import utilities as uf


class Dataset(object):
    def __init__(self, file_name, dataset=None, instance=None, nb_trip_per_district=None, nb_vehicle_per_district=None):
        self.file_name = file_name
        if dataset is None:
            dataset = []
        if instance is None:
            instance = []
        if nb_trip_per_district is None:
            nb_trip_per_district = []
        if nb_vehicle_per_district is None:
            nb_vehicle_per_district = []
        self.dataset = dataset
        self.instance = instance
        self.nb_requests = 0
        self.max_capacity = 0
        self.origin = dt.utcnow()
        self.nb_trip_per_district = nb_trip_per_district
        self.start_hr = 7
        self.start_min = 0
        self.end_hr = 9
        self.end_min = 0
        self.nb_vehicles = 2000
        self.nb_vehicle_per_district = nb_vehicle_per_district
        self.nb_customers = 0

    def read_dataset_data(self):
        """ read data """
        file = open(c.DAYS_DIR + self.file_name + ".json")
        df_trips = json.load(file)
        file.close()
        main_trip_data = []
        for i in range(len(df_trips)):
            main_trip_data.append([df_trips[i]['passenger_count'], df_trips[i]['origin_cell'],
                                   df_trips[i]['destination_cell'], df_trips[i]['request_time'],
                                   df_trips[i]['tpep_dropoff_datetime'], df_trips[i]['origin']['lat'],
                                   df_trips[i]['origin']['lon'], df_trips[i]['destination']['lat'],
                                   df_trips[i]['destination']['lon']])
        self.dataset = pd.DataFrame(main_trip_data, columns=['passenger_count', 'pickup_ID', 'dropoff_ID',
                                                             'tpep_pickup_datetime', 'tpep_dropoff_datetime',
                                                             'pickup_latitude', 'pickup_longitude',
                                                             'dropoff_latitude', 'dropoff_longitude'])
        """ sort data """
        self.dataset = self.dataset.sort_values(by=['tpep_pickup_datetime'])
        self.update_state()
        self.convert_to_datetime()
        year = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).year
        month = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).month
        day = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).day
        self.origin = dtime.datetime(year, month, day, 0, 0, 0)

    def convert_to_datetime(self):
        self.dataset['tpep_pickup_datetime'] = self.dataset['tpep_pickup_datetime'].apply(dt.fromtimestamp)
        self.dataset['tpep_dropoff_datetime'] = self.dataset['tpep_dropoff_datetime'].apply(dt.fromtimestamp)
        self.dataset['tpep_pickup_datetime'] = pd.to_datetime(self.dataset['tpep_pickup_datetime'])
        self.dataset['tpep_pickup_datetime'] = pd.to_datetime(self.dataset['tpep_pickup_datetime'])

    """ function to calculate datetime fields based on seconds from an origin datetime and calculate
     min travel time in seconds to dataframe"""

    def calculate_times_seconds(self):
        self.dataset['request_time_sec'] = (self.dataset['tpep_pickup_datetime'] - self.origin).dt.seconds
        self.instance = self.dataset

    def remove_unwanted_columns_instance(self):
        if 'tpep_pickup_datetime' in self.instance.columns:
            self.instance = self.instance.drop(columns=['tpep_pickup_datetime', 'tpep_dropoff_datetime'])
 #       if 'dropoff_district' in self.instance.columns:
 #           self.instance = self.instance.drop(columns=['dropoff_district'])
        #            self.instance = self.instance.drop(columns=['pickup_district'])
        if 'pickup_latitude' in self.instance.columns:
            self.instance.drop(columns=['pickup_latitude', 'pickup_longitude', 'dropoff_latitude', 'dropoff_longitude'])
        self.instance = self.instance[
            ['passenger_count', 'pickup_ID', 'dropoff_ID', 'request_time_sec', 'pickup_district', 'dropoff_district']]

    def limit_time_instance(self, start_hr=None, end_hr=None, start_min=None, end_min=None):
        if start_hr is not None:
            self.start_hr = start_hr
        if end_hr is not None:
            self.end_hr = end_hr
        if start_min is not None:
            self.start_min = start_min
        if end_min is not None:
            self.end_min = end_min

        start_seconds, end_seconds = uf.calculate_time_from_origin_sec(self.origin, self.start_hr, self.end_hr,
                                                                       self.start_min, self.end_min)
        self.calculate_times_seconds()
        self.instance = self.instance[self.instance.request_time_sec >= start_seconds]
        self.instance = self.instance[self.instance.request_time_sec < end_seconds]
        print("\nThe number of data records after time limit:", len(self.instance.index))
        print("\nThe number of passengers:", self.nb_customers)

    def limit_time_dataset(self, start_hr=None, end_hr=None, start_min=None, end_min=None):
        if start_hr is not None:
            self.start_hr = start_hr
        if end_hr is not None:
            self.end_hr = end_hr
        if start_min is not None:
            self.start_min = start_min
        if end_min is not None:
            self.end_min = end_min
        start_seconds, end_seconds = uf.calculate_time_from_origin_sec(self.origin, self.start_hr, self.end_hr,
                                                                       self.start_min, self.end_min)
        self.calculate_times_seconds()
        self.dataset = self.dataset[self.dataset.request_time_sec >= start_seconds]
        self.dataset = self.dataset[self.dataset.request_time_sec < end_seconds]
        self.update_state()
        self.instance = self.dataset

    def update_state(self):
        self.nb_requests = len(self.dataset.index)
        self.max_capacity = self.dataset.max(numeric_only=True)['passenger_count']
        self.nb_customers = self.dataset.sum(numeric_only=True)['passenger_count']

    def save_dataset(self):
        folder_name = c.DAYS_DIR + "Main"
        if not os.path.exists(folder_name):
            os.mkdir(folder_name)
        trip_file = folder_name + "/" + self.file_name + ".csv"
        # write trip data to csv
        self.dataset.to_csv(trip_file, index=False)

    def limit_capacity(self, capacity):
        self.dataset = self.dataset[self.dataset.passenger_count <= capacity]
        self.update_state()
        print("\nThe number of data records based on capacity:", self.nb_requests)
        print("The number of passengers:", self.nb_customers)

    """ this function split ride requests with number of passengers more than the vehicle capacity"""

    def split_requests(self, capacity):
        removed_rows = self.dataset[self.dataset.passenger_count > capacity]
        self.dataset = self.dataset[self.dataset.passenger_count <= capacity]
        temp_df = []
        num_passengers = []
        for row in removed_rows.itertuples(index=False):
            num_rows = row.passenger_count // capacity
            if row.passenger_count % capacity > 0:
                num_rows = num_rows + 1
            temp_df.extend([list(row)] * num_rows)
            new_passenger = row.passenger_count // num_rows
            num_passengers.extend([new_passenger] * (num_rows - 1))
            num_passengers.extend([row.passenger_count - (new_passenger * (num_rows - 1))] * 1)
        #
        # #            temp_df.extend([list(row)] * (row.passenger_count // capacity))
        #             temp_df.extend([list(row)] * num_rows)
        #             num_passengers.extend([row.passenger_count // num_rows] * num_rows)
        # #            num_passengers.extend([capacity] * (row.passenger_count // capacity))
        #             if row.passenger_count % capacity > 0:
        #                 temp_df.extend([list(row)] * 1)
        #                 num_passengers.extend([(row.passenger_count % capacity)] * 1)
        df_removed_rows = pd.DataFrame(temp_df, columns=self.dataset.columns)
        df_removed_rows["passenger_count"] = pd.DataFrame(num_passengers, columns=['passenger_count'])
        self.dataset = pd.concat([self.dataset, df_removed_rows])
        self.dataset = self.dataset.sort_values(by=['tpep_pickup_datetime'])
        self.update_state()
        self.instance = self.dataset
        print("The number of trips after split:", self.nb_requests)

    def add_district_id(self, network):
        pickup_id = []
        dropoff_id = []
        for row in self.dataset.itertuples(index=False):
            pickup_id.append(network.cell_to_district[row[1]])
            dropoff_id.append(network.cell_to_district[row[2]])
        self.dataset['pickup_district'] = pickup_id
        self.dataset['dropoff_district'] = dropoff_id

    """ this function that keep the points that are inside the desired polygon"""

    def restrict_to_district(self, districts):
        temp_df = []
        for item in districts:
 #           selected_records = self.dataset[self.dataset.pickup_district == item.cartodb_id]
            selected_records = self.dataset[self.dataset.pickup_district == item]
            temp_df.append(selected_records)
        temp_df = pd.concat(temp_df)
        self.dataset = pd.DataFrame(temp_df)

        temp_df = []
        for item in districts:
            selected_records = self.dataset[self.dataset.dropoff_district == item]
            temp_df.append(selected_records)
        temp_df = pd.concat(temp_df)
        self.dataset = pd.DataFrame(temp_df)

        self.dataset = self.dataset.sort_values(by=['tpep_pickup_datetime'])
        self.update_state()

    def remove_same_pick_drop(self):
        for index, row in self.dataset.iterrows():
            if row['pickup_ID'] == row['dropoff_ID']:
                self.dataset = self.dataset.drop(index)
        self.update_state()

    def calculate_trip_per_district(self, network):
        total_record = 0
        self.nb_trip_per_district = []
        for item in network.districts:
            selected_records = self.dataset[self.dataset.pickup_district == item.cartodb_id]
            self.nb_trip_per_district.append(len(selected_records))
            total_record = total_record + len(selected_records)
        """ find out bound requests"""

    #        selected_records = self.dataset[self.dataset.pickup_district == c.OUT_BOUND]
    #        self.nb_trip_per_district.append(len(selected_records))

    def calculate_vehicle_per_district(self, network, nb_vehicles=None):
        if nb_vehicles is not None:
            self.nb_vehicles = nb_vehicles
        self.nb_vehicle_per_district.clear()
        total = 0
        for count, item in enumerate(network.districts):
            self.nb_vehicle_per_district.append(
                round((self.nb_vehicles * self.nb_trip_per_district[count]) / self.nb_requests))
            total = total + self.nb_vehicle_per_district[count]
        self.nb_vehicle_per_district.append(
            round((self.nb_vehicles * self.nb_trip_per_district[-1]) / self.nb_requests))
        total = total + self.nb_vehicle_per_district[len(self.nb_vehicle_per_district) - 1]
        if total != self.nb_vehicles:
            self.nb_vehicle_per_district[0] = self.nb_vehicle_per_district[0] + self.nb_vehicles - total

    def prepare_dataset(self, capacity=None, network=None, start_hr=None, end_hr=None, start_min=None, end_min=None):
        self.read_dataset_data()
        if network is not None:
            self.add_district_id(network)
            self.calculate_trip_per_district(network)
        if start_hr is not None:
            self.limit_time_dataset(start_hr, end_hr, start_min, end_min)
        if capacity is not None:
            self.split_requests(capacity)

    def save_instance(self, nb_vehicles=None):
        period_start, period_end = uf.calculate_time_from_origin(self.origin, self.start_hr, self.end_hr,
                                                                 self.start_min, self.end_min)
        start_seconds, end_seconds = uf.calculate_time_from_origin_sec(self.origin, self.start_hr, self.end_hr,
                                                                       self.start_min, self.end_min)

        file_name = period_start.strftime("%Y%m%d") + "_" + period_start.strftime("%H") + "-" + str(
            int((period_end - period_start).seconds / 60)) + "m"
        self.file_name = period_start.strftime("%Y%m%d")

        # instance folder
        parent_folder = c.DAYS_DIR + "Instances"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        instance_dir = parent_folder + "/"

        if not os.path.exists(instance_dir + file_name):
            os.mkdir(instance_dir + file_name)
        txt_file = instance_dir + file_name + "/" + "TRIP_" + file_name + ".txt"
        csv_file = instance_dir + file_name + "/" + "TRIP_" + file_name + ".csv"
        instance_file = instance_dir + file_name + "/" + "INSTANCE_" + file_name + ".txt"

        self.remove_unwanted_columns_instance()
        df_columns = self.instance.columns.tolist()

        # write trip data to txt
        file = open(txt_file, "w")
        file.write("COLUMNS\n\n")

        for col in df_columns:
            file.write(col)
            file.write("\n")
        file.write("\nREQUESTS_INFO\n")
        df_as_string = self.instance.to_string(header=False, index=False)
        file.write(df_as_string)
        file.close()

        # write instance data to txt
        file = open(instance_file, "w")
        file.write("INSTANCE = ")
        file.write(file_name)
        file.write("\n\n")

        file.write("SIMULATION_START = ")
        file.write(str(start_seconds))
        file.write("\n\n")

        file.write("NUM_VEHICLES = ")
        if nb_vehicles is not None:
            file.write(str(nb_vehicles))
        else:
            file.write(str(2000))
        file.write("\n")
        file.write("NUM_ONBOARDS = ")
        file.write(str(0))
        file.write("\n")
        file.write("NUM_RECEIVED = ")
        file.write(str(0))
        file.write("\n")
        file.write("NUM_REQUESTS = ")
        file.write(str(len(self.instance.index)))
        file.write("\n")
        file.write("NUM_LOCATIONS = ")
        file.write(str(1718))
        file.close()
        # write trip data to csv
        self.instance.to_csv(csv_file, index=False)

    def limit_capacity(self, capacity):
        self.dataset = self.dataset[self.dataset.passenger_count <= capacity]
        self.update_state()
        print("\nThe number of data records based on capacity:", self.nb_requests)

    def visualize_dataset(self, network):
        vf.plot_map_request_cells(district_network=network, dataset=self, print_id=False, file_name=self.file_name)
        vf.plot_districts_by_nb_trips(trip_per_district=self.nb_trip_per_district, district_network=network,
                                      print_id=True, file_name=self.file_name)
