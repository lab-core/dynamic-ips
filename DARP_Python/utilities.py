import json
import pandas as pd
import os
import constants as c
import datetime as dtime


def create_vehicle_dataset(vehicle_file):
    # read data
    f = open(c.VEHICLES_DIR + vehicle_file + ".json")
    df_vehicles = json.load(f)
    f.close()

    # covert data to matrix
    vehicle_data = []
    for i in range(len(df_vehicles)):
        vehicle_data.append(
            [i, df_vehicles[i]['capacity'], df_vehicles[i]['start_time'], 90000, df_vehicles[i]['start_stop_id'],
             df_vehicles[i]['start_stop_id']])
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


def calculate_time_from_origin(time_origin, start_hr, end_hr):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    period_start = dtime.datetime(year, month, day, start_hr, 0, 0)
    period_end = dtime.datetime(year, month, day, end_hr, 0, 0)
    return period_start, period_end


def calculate_time_from_origin_sec(time_origin, start_hr, end_hr):
    period_start, period_end = calculate_time_from_origin(time_origin, start_hr, end_hr)
    start_seconds = (period_start - time_origin).total_seconds()
    end_seconds = (period_end - time_origin).total_seconds()
    return start_seconds, end_seconds

