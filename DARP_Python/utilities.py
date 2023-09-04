import json
import pandas as pd
import os
import constants as c
import datetime as dtime
import visualize as vf
from Vehicle import *
import glob
from pathlib import Path


def create_restricted_vehicle_dataset(vehicle_file, districts, network):
    # read data
    f = open(c.VEHICLES_DIR + vehicle_file + ".json")
    df_vehicles = json.load(f)
    f.close()
    source_ids = []
    for item in districts:
        for cell in item.cells:
            source_ids.append(int(cell[0]))
    # covert data to matrix
    vehicle_data = []
    for i in range(len(df_vehicles)):
        source_id = source_ids[i % len(source_ids)]
        zone_id = network.cell_to_district[int(source_id)]
        vehicle_data.append(
            [i, df_vehicles[i]['capacity'], df_vehicles[i]['start_time'], 90000, source_id, source_id, zone_id])

    df_vehicles = pd.DataFrame(vehicle_data,
                               columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time', 'depart_ID', 'sink_ID',
                                        'zone_ID'])
    vf.plot_map_vehicle_cells(network, df_vehicles, print_id=False, file_name=vehicle_file)
    # save data file
    folder_name = c.VEHICLES_DIR + "limited_manhattan-vehicles"
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)
    file_to_save = folder_name + "/" + vehicle_file + ".txt"
    df_columns = df_vehicles.columns.tolist()
    file = open(file_to_save, "w")
    file.write("COLUMNS\n\n")

    for col in df_columns:
        file.write(col)
        file.write("\n")
    file.write("\nVEHICLES_INFO\n")
    df_as_string = df_vehicles.to_string(header=False, index=False)
    file.write(df_as_string)
    file.close()


def create_vehicle_dataset_with_zone(vehicle_file, network):
    # read data
    f = open(c.VEHICLES_DIR + vehicle_file + ".json")
    df_vehicles = json.load(f)
    f.close()
    source_ids = []
    for item in network.districts:
        for cell in item.cells:
            source_ids.append(int(cell[0]))
    # covert data to matrix
    vehicle_data = []
    for i in range(len(df_vehicles)):
        source_id = df_vehicles[i]['start_stop_id']
        zone_id = network.cell_to_district[int(source_id)]
        vehicle_data.append(
            [i, df_vehicles[i]['capacity'], df_vehicles[i]['start_time'], 90000, source_id, source_id, zone_id])

    df_vehicles = pd.DataFrame(vehicle_data,
                               columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time', 'depart_ID', 'sink_ID',
                                        'zone_ID'])
    vf.plot_map_vehicle_cells(network, df_vehicles, print_id=False, file_name=vehicle_file)

    # save data file
    folder_name = c.VEHICLES_DIR + "manhattan-vehicles"
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)
    file_to_save = folder_name + "/" + vehicle_file + ".txt"
    df_columns = df_vehicles.columns.tolist()
    file = open(file_to_save, "w")
    file.write("COLUMNS\n\n")

    for col in df_columns:
        file.write(col)
        file.write("\n")
    file.write("\nVEHICLES_INFO\n")
    df_as_string = df_vehicles.to_string(header=False, index=False)
    file.write(df_as_string)
    file.close()


def create_vehicle_dataset_zone(vehicle_file, districts, nb_per_zone):
    # read data
    f = open(c.VEHICLES_DIR + vehicle_file + ".json")
    df_vehicles = json.load(f)
    f.close()
    # covert data to matrix
    vehicle_data = []
    vehicle_id = 0
    for count, item in enumerate(districts):
        source_ids = []
        for cell in item.cells:
            source_ids.append(int(cell[0]))
        source_ids = sorted(source_ids)
        for i in range(nb_per_zone[count]):
            source_id = source_ids[i % len(source_ids)]
            vehicle_data.append(
                [vehicle_id, df_vehicles[i]['capacity'], df_vehicles[i]['start_time'], 90000, source_id, source_id])
            vehicle_id = vehicle_id + 1

    df_vehicles = pd.DataFrame(vehicle_data,
                               columns=['vehicle_ID', 'capacity', 'depart_Time', 'end_Time', 'depart_ID', 'sink_ID'])

    # save data file
    folder_name = c.VEHICLES_DIR + "manhattan-vehicles"
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)
    file_to_save = folder_name + "/" + vehicle_file + ".txt"
    df_columns = df_vehicles.columns.tolist()
    file = open(file_to_save, "w")
    file.write("COLUMNS\n\n")

    for col in df_columns:
        file.write(col)
        file.write("\n")
    file.write("\nVEHICLES_INFO\n")
    df_as_string = df_vehicles.to_string(header=False, index=False)
    file.write(df_as_string)
    file.close()


def create_file_names():
    file_names = []
    for item in c.DATES_2015:
        file_names.append("2015-" + item + "_manhattan")
    for item in c.DATES_2016:
        file_names.append("2016-" + item + "_manhattan")
    return file_names


def create_vehicle_file_names():
    file_names = []
    for num in c.NB_VEHICLES:
        for cap in c.CAPACITY:
            file_names.append("vehicles_" + num + "_" + cap)
    return file_names


def calculate_time_from_origin(time_origin, start_hr, end_hr, start_min, end_min):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    period_start = dtime.datetime(year, month, day, start_hr, start_min, 0)
    period_end = dtime.datetime(year, month, day, end_hr, end_min, 0)
    return period_start, period_end


def calculate_time_from_origin_sec(time_origin, start_hr, end_hr, start_min, end_min):
    period_start, period_end = calculate_time_from_origin(time_origin, start_hr, end_hr, start_min, end_min)
    start_seconds = (period_start - time_origin).total_seconds()
    end_seconds = (period_end - time_origin).total_seconds()
    return start_seconds, end_seconds


def create_vehicles_from_files(network, selected_districts=None, replace=False):
    for file in glob.glob(c.VEHICLES_DIR + "*.json"):
        vehicle_obj = Vehicle(len(network.districts), file_name=Path(file).stem)
        vehicle_obj.create_vehicle_data_from_file(network=network, selected_districts=selected_districts,
                                                  replace=replace)
        if selected_districts is not None:
            vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="limited_vehicles_plots")
            vehicle_obj.save_vehicle(folder_name="limited_manhattan-vehicles")
        else:
            vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="vehicles_plots")
            vehicle_obj.save_vehicle(folder_name="manhattan-vehicles")


def create_vehicles_files(network, initial_vehicle):
    # for num in c.NB_VEHICLES:
    #     for cap in c.CAPACITY:
    #         file = "vehicles_" + str(num) + "_" + str(cap)
    #         vehicle_obj = Vehicle(len(network.districts), nb_trip_per_district=initial_vehicle.nb_trip_per_district,
    #                               file_name=file, nb_vehicles=num, capacity=cap)
    #         vehicle_obj.calculate_vehicle_per_district()
    #         vehicle_obj.create_vehicle_data_from_districts(network=network)
    #         if cap == 4:
    #             vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="sufficient_vehicles_plots")
    #         vehicle_obj.save_vehicle(folder_name="sufficient_manhattan-vehicles")

    nb_vehicles = c.NB_VEHICLES
    initial_total = nb_vehicles.pop()
    for cap in c.CAPACITY:
        file = "vehicles_" + str(initial_total) + "_" + str(cap)
        vehicle_obj = Vehicle(len(network.districts), nb_trip_per_district=initial_vehicle.nb_trip_per_district,
                              file_name=file, nb_vehicles=initial_total, capacity=cap)
        vehicle_obj.calculate_vehicle_per_district()
        vehicle_obj.create_vehicle_data_from_districts(network=network)
        if cap == 4:
            vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="sufficient_vehicles_plots")
        vehicle_obj.save_vehicle(folder_name="sufficient_manhattan-vehicles")

    df = vehicle_obj.vehicle_data
    while nb_vehicles:
        num = nb_vehicles.pop()
        counts = df['zone_ID'].value_counts().to_dict()

        total = 0
        new_counts = {}
        n_to_remove = {}
        for key, value in counts.items():
            new_counts[key] = round((num * value) / initial_total)
            total = total + new_counts[key]
        if total != num:
            new_counts[189] = new_counts[189] + num - total

        for key, value in new_counts.items():
            n_to_remove[key] = counts[key] - new_counts[key]

        # Iterate over each group in the DataFrame
        to_drop = []
        for key, value in n_to_remove.items():
            # Select n rows randomly from the current group
            selected = df.loc[df['zone_ID'] == key].sample(value)
            # Append the index of the selected rows to the list of rows to drop
            to_drop.extend(selected.index)

        # Drop the selected rows from the DataFrame
        df = df.drop(to_drop)

        # correct ids
        v_id = 0
        for index, row in df.iterrows():
            row['vehicle_ID'] = v_id
            v_id = v_id + 1

        vehicle_obj.vehicle_data = df
        for cap in c.CAPACITY:
            file = "vehicles_" + str(num) + "_" + str(cap)
            vehicle_obj.vehicle_data['capacity'] = cap
            vehicle_obj.file_name = file
            vehicle_obj.nb_vehicles = num
            vehicle_obj.save_vehicle(folder_name="sufficient_manhattan-vehicles")
            if cap == 4:
                vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="sufficient_vehicles_plots")

        initial_total = num


def create_vehicles_files_random_remove(network, file_name):
    df = pd.read_csv(c.VEHICLES_DIR + file_name)
    # count the number of vehicles in zones
    nb_vehicles = [500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900]
    initial_total = 2000
    while nb_vehicles:
        num = nb_vehicles.pop()
        counts = df['zone_ID'].value_counts().to_dict()
        file = "vehicles_" + str(num) + "_" + str(4)
        vehicle_obj = Vehicle(len(network.districts), file_name=file, nb_vehicles=num, capacity=4)

        total = 0
        new_counts = {}
        n_to_remove = {}
        for key, value in counts.items():
            new_counts[key] = round((num * value) / initial_total)
            total = total + new_counts[key]
        if total != num:
            new_counts[189] = new_counts[189] + num - total

        for key, value in new_counts.items():
            n_to_remove[key] = counts[key] - new_counts[key]

        # Iterate over each group in the DataFrame
        to_drop = []
        for key, value in n_to_remove.items():
            # Select n rows randomly from the current group
            selected = df.loc[df['zone_ID'] == key].sample(value)
            # Append the index of the selected rows to the list of rows to drop
            to_drop.extend(selected.index)

        # Drop the selected rows from the DataFrame
        df = df.drop(to_drop)

        # correct ids
        v_id = 0
        for index, row in df.iterrows():
            row['vehicle_ID'] = v_id
            v_id = v_id + 1

        vehicle_obj.vehicle_data = df
        vehicle_obj.plot_map_vehicle_cells(network, print_id=False, folder_name="sufficient_new_vehicles_plots")
        vehicle_obj.save_vehicle(folder_name="sufficient_new_manhattan-vehicles")
        initial_total = num


def merge_csv_files(instance_folder):
    root_folder = c.DATASETS_DIR + instance_folder
    all_data = pd.DataFrame()
    header_added = False

    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith(".csv") and file.startswith("summary"):

                file_path = os.path.join(root, file)
                data = pd.read_csv(file_path, index_col=False)
                parent_folder = os.path.basename(root)
                parent_folder_col = pd.Series([parent_folder] * len(data))
                data['Test_Folder'] = parent_folder_col
                all_data = pd.concat([all_data, data], ignore_index=True)

    root_file = root_folder + "/" + "results.csv"
    all_data.to_csv(root_file, index=False)
    print("CSV files merged and saved to", root_folder)

