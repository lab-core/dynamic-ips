# -*- coding: utf-8 -*-
"""
Created on Mon Jan  3 13:02:45 2022

@author: Ella
"""

import numpy as np
import pandas as pd
import requests
import math
from requests.structures import CaseInsensitiveDict
import os
from copy import deepcopy

# function that determine whether a point is inside a polygon or not
def point_inside_polygon(x,y,polygon):
    n = len(polygon.index)
    poly_matrix = polygon.to_numpy()
    inside = False
    p1x,p1y = poly_matrix[0]
    for i in range(n+1):
        p2x,p2y = poly_matrix[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside

# function that keep the points that are inside the desired polygon
def trip_requests_inside_polygon(df_input ,polygon):
    removed_index = []
    for index, row in df_input.iterrows():
        x_pick , y_pick = row["pickup_latitude"], row["pickup_longitude"]
        x_drop , y_drop = row["dropoff_latitude"], row["dropoff_longitude"]
        if (not point_inside_polygon(x_pick, y_pick, polygon)) or (not point_inside_polygon(x_drop, y_drop, polygon)):
            removed_index.append(index);
    df_limited = df_input.drop(removed_index, axis = 0);
    return df_limited

# function to remove unwanted columns, records with zero values
def remove_unwanted_data(df_input):
    
    # remove unwanted columns
    df_clean = df_input.drop(columns =[
        'VendorID','RatecodeID','store_and_fwd_flag','payment_type','fare_amount','extra', 
        'mta_tax', 'tip_amount', 'tolls_amount', 'improvement_surcharge', 'total_amount', 'trip_distance'])
    
    # remove records with zero value
    for column in df_clean:
        df_clean = df_clean[df_clean[column] != 0]
    return df_clean


# function to filter trip records within desired time period
def filter_records_time_period(df_input, period_start, period_end):
    df_input[['tpep_pickup_datetime','tpep_dropoff_datetime']] = df_input[['tpep_pickup_datetime','tpep_dropoff_datetime']].apply(pd.to_datetime)
    df_input = df_input[df_input.tpep_pickup_datetime >= period_start]
    df_input = df_input[df_input.tpep_pickup_datetime <= period_end]
    return df_input

# function to calculate datetime fileds based on seconds from an origin datetime and calculate min travel time in seconds to dataframe
def calculate_times_seconds(df_input, origin_time):
    df_input['pickup_time_sec'] = (df_input['tpep_pickup_datetime'] - origin_time).dt.seconds
  
    # remove travel time columns
    df_output = df_input.drop(columns =['tpep_pickup_datetime', 'tpep_dropoff_datetime'])
    return df_output

def list_locations(df_data):
    df_pickups = df_data[["pickup_latitude","pickup_longitude"]]
    df_dropoffs = df_data[["dropoff_latitude","dropoff_longitude"]]
    dict = {'pickup_latitude': 'latitude',
            'dropoff_latitude': 'latitude',
            'pickup_longitude': 'longitude',
            'dropoff_longitude': 'longitude'}
    df_pickups.rename(columns=dict, inplace=True)
    df_dropoffs.rename(columns=dict, inplace=True)
    df_result = pd.concat([df_pickups,df_dropoffs],ignore_index = True, axis = 0)
    source = pd.DataFrame({'latitude': [df_result["latitude"].mean()], 
                           'longitude' : [df_result["longitude"].mean()]})
    df_result = pd.concat([df_result, source],ignore_index = True, axis = 0)
    df_result['ID'] = np.arange(len(df_result))
    df_result = df_result[['ID','latitude','longitude']]
    return df_result


# function to perform all required operations on the input data and transfer it
def transfer_dataset(df_input, polygon, period_start, period_end, origin_time):
    df_filtered = filter_records_time_period(df_input, period_start, period_end);
    df_reduced = remove_unwanted_data(df_filtered);
    df_limited = trip_requests_inside_polygon(df_reduced , polygon);
    
    # sort records based on request time
    df_sorted = df_limited.sort_values(by=['tpep_pickup_datetime']);
    
    # calculate datetime fileds based (in seconds) and add min travel time (in seconds) to dataframe
    df_output = calculate_times_seconds(df_sorted, origin_time);
    
    # reordering the columns in dataset and add location IDs
    df_output['pickup_ID'] = np.arange(len(df_output))
    df_output['dropoff_ID'] = np.arange(len(df_output),2*len(df_output))
    df_output = df_output[['passenger_count', 'pickup_ID', 'dropoff_ID','pickup_time_sec', 'pickup_latitude', 'pickup_longitude','dropoff_latitude', 'dropoff_longitude',]]
    
    print("\nThe number of data records in time period is:", len(df_filtered.index))
    print("The number of clean data records is:", len(df_reduced.index))
    print("The number of trips inside desired area is:", len(df_limited.index))
    return df_output

def show_data_situation(dataframe):
    start = "\033[1m"
    end = "\033[0;0m"
    print (start + "situation of dataset: " + end)
    print("\nThe number of data columns is:", len(dataframe.columns))
    print("The number of trip records is:", len(dataframe.index))
    print("\nThe types of data columns are:")
    print(dataframe.dtypes)
    print()
    
def calculate_durations(df_input):
    df_locations = deepcopy(df_input)
    df_locations['coordinate'] = df_locations['longitude'].astype(str)+','+df_locations['latitude'].astype(str)

    locaion_list = df_locations['coordinate'].astype(str).values.flatten().tolist()
    source_list = df_locations['ID'].astype(str).values.flatten().tolist()
    source_index = df_locations['ID'].to_numpy(dtype='int')
    
    # divide data to sets with 100 points due to limitaion of OSRM
    source_split = []
    location_string = []
    source_header = []
    split_size = 100
    part = math.floor(len(locaion_list)/split_size)
    
    for i in range(part):
        source_split.append([source_index[i*split_size:(i+1)*split_size]])
        location_string.append(";".join([np.array(locaion_list)[index] for index in source_split[i]][0]))
        source_header.append([np.array(source_list)[index] for index in source_split[i]])

    # add the remind data
    if part*split_size < len(source_index):
        source_split.append([source_index[part*split_size:len(source_index)]])
        location_string.append(";".join([np.array(locaion_list)[index] for index in source_split[part]][0]))
        source_header.append([np.array(source_list)[index] for index in source_split[part]])
    
    # read durations from OSRM server
    starturl = "http://206.12.92.28/ny/table/v1/driving/"
    headers = CaseInsensitiveDict()
    headers["Accept"] = "application/json"
    duration_Mat = np.empty((0,len(locaion_list)),float)
    for i in range(len(source_split)):
        duration_SubMat = np.empty((len(source_split[i][0]),0),float)
        for j in range(len(source_split)):
            url = starturl + location_string[i] + ";" + location_string[j]
            source_str = ";".join(source_list[0:len(source_split[i][0])])
            destination_str = ";".join(np.arange(len(source_split[i][0]),len(source_split[i][0])+len(source_split[j][0])).astype(str))
            url = url + "?sources=" + source_str + "&destinations=" + destination_str + "&generate_hints=false"
            resp = requests.get(url, headers=headers)
            duration_SubMat = np.concatenate((duration_SubMat, np.array(resp.json()['durations'])), axis=1)
        duration_Mat = np.concatenate((duration_Mat, duration_SubMat), axis=0)

            
    # covert data to matrix
    duration_data = []
    for i in range(duration_Mat.shape[0]):
        for j in range(duration_Mat.shape[1]):
            duration_data.append([i, j, duration_Mat[i][j]])
    df_durations = pd.DataFrame(duration_data, columns =['startID', 'EndID', 'duration'])
    return df_durations
    
# function to print duration data in a txt file
def printDurations(fileName, df_durations):
    if not os.path.exists(fileName):
        os.mkdir(fileName)
    durationFile = fileName + "/" + "DURATION_" + fileName + ".txt"
    
    file = open(durationFile, mode='w')
    file.write("COLUMNS\n\n")
    df_columns = df_durations.columns.tolist()

    for col in df_columns:
        file.write(col)
        file.write("\n")
    file.write("\nDURATION_INFO\n")
    file.close()
    file = open(durationFile,'a')
    np.savetxt(file, df_durations.values, fmt='%d    %d    %1.1f')
    file.close() 
    
# function to print instance data in a txt file
def printData(fileName, df_data, df_coordinates, nbVehicles, capacity, start, end):
    df_columns = df_data.columns.tolist()
    if not os.path.exists(fileName):
        os.mkdir(fileName)
    txtFile = fileName + "/" + "TRIP_" + fileName + ".txt"
    csvFile = fileName + "/" + "TRIP_" + fileName + ".csv"
    instanceFile = fileName + "/" + "INSTANCE_" + fileName + ".txt"
    locationFile = fileName + "/" + "LOCATION_" + fileName + ".csv"
    
    sourceLatitude = (df_data["pickup_latitude"].mean() + df_data["dropoff_latitude"].mean())/2
    sourceLongitude = (df_data["pickup_longitude"].mean() + df_data["dropoff_longitude"].mean())/2

    
    # write trip data to txt
    file = open(txtFile, "w")
    file.write("COLUMNS\n\n")
    
    for col in df_columns:
        file.write(col)
        file.write("\n")
    file.write("\nREQUESTS_INFO\n")
    dfAsString = df_data.to_string(header=False, index=False)
    file.write(dfAsString)
    file.close()
    
    # write instance data to txt
    file = open(instanceFile, "w")
    file.write("INSTANCE = ")
    file.write(fileName)
    file.write("\n\n")
    
    file.write("NUM_VEHICLES = ")
    file.write(str(nbVehicles))
    file.write("\n\n")
    
    file.write("NUM_REQUESTS = ")
    file.write(str(len(df_data.index)))
    file.write("\n\n")
    
    file.write("SOURCE_ID = ")
    file.write(str(2*len(df_data.index)))
    file.write("\n")
    file.write("SINK_ID = ")
    file.write(str(2*len(df_data.index)))
    file.write("\n\n")
    
    file.write("SOURCE_LATITUDE = ")
    file.write("{:.12f}".format(sourceLatitude))
    file.write("\n")
    file.write("SOURCE_LONGITUDE = ")
    file.write("{:.12f}".format(sourceLongitude))
    file.write("\n\n")
    
    file.write("SINK_LATITUDE = ")
    file.write("{:.12f}".format(sourceLatitude))
    file.write("\n")
    file.write("SINK_LONGITUDE = ")
    file.write("{:.12f}".format(sourceLongitude))
    file.write("\n\n")    
    
    file.write("VEHICLES_INFO\n")
    file.write(str(0))
    file.write("\t")
    file.write(str(capacity))
    file.write("\t")
    file.write(str(start))
    file.write("\t")
    file.write(str(end))
    file.write("\n")
    file.close()
    
    # write trip data to csv
    df_data.to_csv(csvFile, index=False)
    
    #write locatios data to csv
    df_coordinates.to_csv(locationFile, index=False)

