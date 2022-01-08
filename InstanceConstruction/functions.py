# -*- coding: utf-8 -*-
"""
Created on Mon Jan  3 13:02:45 2022

@author: Ella
"""

import numpy as np
import pandas as pd
import datetime as dt
import os

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

# function to perform all required operations on the input data and transfer it
def transfer_dataset(df_input, polygon, period_start, period_end, origin_time):
    df_filtered = filter_records_time_period(df_input, period_start, period_end);
    df_reduced = remove_unwanted_data(df_filtered);
    df_limited = trip_requests_inside_polygon(df_reduced , polygon);
    
    # sort records based on request time
    df_sorted = df_limited.sort_values(by=['tpep_pickup_datetime']);
    
    # calculate datetime fileds based (in seconds) and add min travel time (in seconds) to dataframe
    df_output = calculate_times_seconds(df_sorted, origin_time);
    
    # reordering the columns in dataset
    df_output = df_output[['passenger_count', 'pickup_latitude', 'pickup_longitude', 'dropoff_latitude', 'dropoff_longitude','pickup_time_sec']]
    
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
    
    
# function to print instance data in a txt file
def printData(fileName, df_data, nbVehicles, capacity, start, end):
    df_columns = df_data.columns.tolist()
    if not os.path.exists(fileName):
        os.mkdir(fileName)
    txtFile = fileName + "/" + "TR_" + fileName + ".txt"
    csvFile = fileName + "/" + "TR_" + fileName + ".csv"
    instanceFile = fileName + "/" + "IN_" + fileName + ".txt"
    
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
    for v in range(nbVehicles):
        file.write(str(v))
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