import numpy as np
import pandas as pd
import datetime as dt

# ************************** Functions *********************************** #

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
        flag = True
        if (not point_inside_polygon(x_pick,y_pick,df_polygon)) or (not point_inside_polygon(x_drop,y_drop,df_polygon)):
            removed_index.append(index);
    df_limited = df_input.drop(removed_index, axis = 0);
    return df_limited


# function to remove unwanted columns, records with zero values
def remove_unwanted_data(df_input):
    
    # remove unwanted columns
    df_clean = df_input.drop(columns =[
        'VendorID','RatecodeID','store_and_fwd_flag','payment_type','fare_amount','extra', 
        'mta_tax', 'tip_amount', 'tolls_amount', 'improvement_surcharge', 'total_amount'])
    
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

# function to perform all required operations on the input data and transfer it
def transfer_dataset(df_input, polygon, period_start, period_end):
    df_filtered = filter_records_time_period(df_input, period_start, period_end);
    df_reduced = remove_unwanted_data(df_filtered);
    df_limited = trip_requests_inside_polygon(df_reduced , polygon.to_numpy());
    
    # sort records based on request time
    df_output = df_limited.sort_values(by=['tpep_pickup_datetime'])
    
    # add min travel time to dataframe
    df_output['min_travel_time'] = (df_output['tpep_dropoff_datetime'] - df_output['tpep_pickup_datetime']).dt.seconds 
    
    print("\nThe number of data records in time period is:", len(df_filtered.index))
    print("The number of clean data records is:", len(df_reduced.index))
    print("The number of trips inside desired area is:", len(df_limited.index))
    return df_output


def show_data_situation(dataframe):
    print("\nThe number of data columns is:", len(dataframe.columns))
    print("The number of trip records is:", len(dataframe.index))
    print("\nThe types of data columns are:")
    print(dataframe.dtypes)
    print()

# ************************** Reading Data *********************************** #
print ("READING DATA: ")
print ("...........")
# Importing trip record data and polygon points of desired area
trip_filename = "yellow_tripdata_2015-07.csv"
polygon_filename = "Manhattan_polygon_neigh1-6.csv"
df_tripdata = pd.read_csv(trip_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)

# determine the desired period of time for trip requests
period_start = dt.datetime(2015,7,1,16,0,0)
period_end = dt.datetime(2015,7,1,17,0,0)
print("\n====================================================================")
print ("SITUATION OF DATASET BEFORE TRANSFER: ")
show_data_situation(df_tripdata)

# ************************** Transfering Data *********************************** #
df_instance = transfer_dataset(df_tripdata, df_polygon, period_start, period_end)

print("\n====================================================================")
print ("SITUATION OF DATASET AFTER TRANSFER: ")
show_data_situation(df_instance)
print()
print(df_instance.head(5))
file_name = "tripdata_" + period_start.strftime("%Y-%m-%d")+"_"+period_start.strftime("%H")+"-"+period_end.strftime("%H")+".csv"
df_instance.to_csv(file_name)