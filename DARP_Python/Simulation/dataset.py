import json
from datetime import datetime as dt
import datetime as dtime
import Visualization.visualize_dataset as ivf
import pandas as pd
import Simulation.utilities as uf
import constants as c
import os


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
        file = open(c.TRANSFORM_DAY_DIR + self.file_name + ".json")
        df_trips = json.load(file)
        file.close()

        # Remove None/null entries
        df_trips = [trip for trip in df_trips if trip is not None]

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
        self.dataset.sort_values(by=['tpep_pickup_datetime'], kind="mergesort", inplace=True)
        self.update_state()
        self.convert_to_datetime()
        year = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).year
        month = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).month
        day = pd.to_datetime(self.dataset['tpep_pickup_datetime'][0]).day
        self.origin = dtime.datetime(year, month, day, 0, 0, 0)

    def prepare_dataset(self, parent_dir, capacity=None, network=None, start_hr=None, end_hr=None, start_min=None, end_min=None,
                        restrict=None, visualize=False, remove_same=False):
        self.read_dataset_data()

        if network is not None:
            self.add_district_id(network)

        if remove_same:
            self.remove_same_pick_drop(network=network)

        if capacity is not None:
            self.split_requests(capacity)


        if visualize:
            ivf.plot_requests_arrival(self.dataset, parent_dir, self.origin)
            ivf.plot_zoom_request_arrival_4h(self.dataset, parent_dir, self.origin)
            ivf.plot_customer_arrival(self.dataset, parent_dir, self.origin)

        if start_hr is not None:
            self.limit_time_dataset(start_hr, end_hr, start_min, end_min)
        self.calculate_trip_per_district(network)
        average_requests, average_customers = self.compute_average_requests()
        if visualize:
            ivf.plot_map_request_cells(district_network=network, dataset=self.dataset, parent_folder=parent_dir,
                                       file_name=self.file_name)
            ivf.plot_districts_fill(district_network=network, trip_per_district=self.nb_trip_per_district,
                                    parent_folder=parent_dir, file_name=self.file_name)

        print("\nThe number of trips after time limit:", self.nb_requests)
        print("The number of customers after time limit:", self.nb_customers)
        self.calculate_trip_per_district(network)
        #       self.visualize_dataset(network)
        self.save_dataset(parent_dir)
        self.save_instance(parent_dir)
        return average_requests

    def add_district_id(self, network):
        pickup_id = []
        dropoff_id = []
        for row in self.dataset.itertuples(index=False):
            pickup_id.append(network.cell_to_district[row[1]])
            dropoff_id.append(network.cell_to_district[row[2]])
        self.dataset['pickup_district'] = pickup_id
        self.dataset['dropoff_district'] = dropoff_id


    def update_state(self):
        self.nb_requests = len(self.dataset.index)
        self.max_capacity = self.dataset.max(numeric_only=True)['passenger_count']
        self.nb_customers = self.dataset.sum(numeric_only=True)['passenger_count']

    def convert_to_datetime(self):
        self.dataset['tpep_pickup_datetime'] = self.dataset['tpep_pickup_datetime'].apply(dt.fromtimestamp)
        self.dataset['tpep_dropoff_datetime'] = self.dataset['tpep_dropoff_datetime'].apply(dt.fromtimestamp)
        self.dataset['tpep_pickup_datetime'] = pd.to_datetime(self.dataset['tpep_pickup_datetime'])
        self.dataset['tpep_pickup_datetime'] = pd.to_datetime(self.dataset['tpep_pickup_datetime'])

    def split_requests(self, capacity):
        """
                    Split any rows whose passenger_count exceeds `capacity` into multiple rows,
                    preserving the original row order. No final sort is needed.

                    Assumes:
                      - self.dataset is a pandas DataFrame with a 'passenger_count' column.
                      - self.update_state() and self.instance are part of the class.
                    """

        cols = list(self.dataset.columns)
        pc_idx = self.dataset.columns.get_loc("passenger_count")
        original_pc_dtype = self.dataset["passenger_count"].dtype

        out_rows = []

        # Iterate once, in order, and append either the original row or its splits.
        for row in self.dataset.itertuples(index=False, name=None):
            row_list = list(row)
            pc = row_list[pc_idx]

            # If passenger_count is missing or not an int-like, just keep the row as-is.
            # (Adjust if you want stricter validation.)
            if pd.isna(pc) or not isinstance(pc, (int, float)):
                out_rows.append(row_list)
                continue

            pc_int = int(pc)

            if pc_int <= capacity:
                out_rows.append(row_list)
                continue

            # Number of rows needed (ceiling division)
            num_rows = (pc_int + capacity - 1) // capacity

            # Distribute passengers as evenly as possible:
            # first (num_rows - 1) rows get `base`, last gets the remainder to sum to pc_int
            base = pc_int // num_rows
            remainder_last = pc_int - base * (num_rows - 1)
            counts = [base] * (num_rows - 1) + [remainder_last]

            for c in counts:
                new_row = row_list.copy()
                new_row[pc_idx] = c
                out_rows.append(new_row)

        # Rebuild the DataFrame in the already-correct order
        self.dataset = pd.DataFrame(out_rows, columns=cols)

        # Restore original dtype for passenger_count if possible
        try:
            self.dataset["passenger_count"] = self.dataset["passenger_count"].astype(original_pc_dtype)
        except Exception:
            # If casting back fails (e.g., original was Int64 and now mixed), keep as-is.
            pass

        # Optional: reset index for cleanliness
        self.dataset.reset_index(drop=True, inplace=True)

        # Keep the rest of your class behavior
        if hasattr(self, "update_state"):
            self.update_state()
        self.instance = self.dataset

        # If you track nb_requests, keep it in sync
        if hasattr(self, "nb_requests"):
            try:
                self.nb_requests = len(self.dataset)
            except Exception:
                pass

        print("The number of trips after split:", len(self.dataset))

    def remove_same_pick_drop(self, network=None):
        duration_dict = network.durations.set_index(['startID', 'EndID'])['duration']

        # Create a boolean mask based on the specified condition
        mask = self.dataset.apply(lambda row: duration_dict.get((row['pickup_ID'], row['dropoff_ID']), 0) > 60, axis=1)

        # Apply the mask to filter the DataFrame
        self.dataset = self.dataset[mask]

        self.update_state()

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

    """ function to calculate datetime fields based on seconds from an origin datetime and calculate
         min travel time in seconds to dataframe"""
    def calculate_times_seconds(self):
        self.dataset['request_time_sec'] = (self.dataset['tpep_pickup_datetime'] - self.origin).dt.seconds
        self.instance = self.dataset

    def calculate_trip_per_district(self, network):
        total_record = 0
        self.nb_trip_per_district = []
        for item in network.districts:
            selected_records = self.dataset[self.dataset.pickup_district == item.cartodb_id]
            self.nb_trip_per_district.append(len(selected_records))
            total_record = total_record + len(selected_records)

    def compute_average_requests(self):
        self.dataset['count'] = 1
        self.dataset['tpep_pickup_datetime'] = pd.to_datetime(self.dataset['tpep_pickup_datetime'])

        # Group data into 30-second intervals and compute total requests
        df_grouped = self.dataset.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='30s')).sum(numeric_only=True)

        # Compute and return the average number of requests and customers per interval
        average_requests = df_grouped['count'].mean()
        average_customers = df_grouped['passenger_count'].mean()
        return average_requests, average_customers

    def save_dataset(self, parent_dir):
        folder_name = parent_dir + "Main"
        os.makedirs(folder_name, exist_ok=True)
        trip_file = folder_name + "/" + self.file_name + ".csv"
        # write trip data to csv
        self.dataset.to_csv(trip_file, index=False)

    def save_instance(self, parent_dir, nb_vehicles=None):
        period_start, period_end = uf.calculate_time_from_origin(self.origin, self.start_hr, self.end_hr,
                                                                 self.start_min, self.end_min)
        start_seconds, end_seconds = uf.calculate_time_from_origin_sec(self.origin, self.start_hr, self.end_hr,
                                                                       self.start_min, self.end_min)

        file_name = period_start.strftime("%Y%m%d") + "_" + period_start.strftime("%H") + "-" + str(
            int((period_end - period_start).seconds / 60)) + "m"
        self.file_name = period_start.strftime("%Y%m%d")

        # instance folder
        parent_folder = parent_dir + "Instances"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        instance_dir = parent_folder + "/"

        if not os.path.exists(instance_dir + file_name):
            os.mkdir(instance_dir + file_name)
        txt_file = instance_dir + file_name + "/" + "TRIP_" + file_name + ".txt"
        csv_file = instance_dir + file_name + "/" + "TRIP_" + file_name + ".csv"
        instance_file = instance_dir + file_name + "/" + "INSTANCE_" + file_name + ".txt"
        requests_file = instance_dir + file_name + "/" + "REQUESTS_" + file_name + ".csv"

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
        file.write(str(1102))
        file.close()
        # write trip data to csv
        self.instance.to_csv(csv_file, index=False)
        self.instance['trip_id'] = range(1, len(self.instance) + 1)
        self.instance.rename(columns={'request_time_sec': 'ready_time', 'pickup_ID': 'origin',
                                      'dropoff_ID': 'destination'}, inplace=True)

        self.instance.drop(columns=['pickup_district', 'dropoff_district'])
        self.instance['release_time'] = self.instance['ready_time']
        self.instance['due_time'] = 100000
        self.instance = self.instance[
            ['trip_id','origin', 'destination', 'release_time', 'ready_time', 'due_time', 'passenger_count']]
        self.instance.to_csv(requests_file, index=False, sep=';')

    def remove_unwanted_columns_instance(self):
        if 'tpep_pickup_datetime' in self.instance.columns:
            self.instance = self.instance.drop(columns=['tpep_pickup_datetime', 'tpep_dropoff_datetime'])

        if 'pickup_latitude' in self.instance.columns:
            self.instance.drop(columns=['pickup_latitude', 'pickup_longitude', 'dropoff_latitude', 'dropoff_longitude'])
        self.instance = self.instance[
            ['passenger_count', 'pickup_ID', 'dropoff_ID', 'request_time_sec', 'pickup_district', 'dropoff_district']]