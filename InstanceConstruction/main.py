# -*- coding: utf-8 -*-
"""
Created on Mon Jan  3 13:06:19 2022

@author: Ella
"""

import functions as fs
import pandas as pd
import datetime as dt


# Importing trip record data and polygon points of desired area

trip_filename = "yellow_tripdata_2015-07.csv"
polygon_filename = "Manhattan_polygon_neigh1-6.csv"
df_tripdata = pd.read_csv(trip_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)

# determine the desired period of time for trip requests
period_start = dt.datetime(2015,7,13,16,0,0)
period_end = dt.datetime(2015,7,13,17,0,0)
origin_time = dt.datetime(2015,7,13,0,0,0)

fs.show_data_situation(df_tripdata)

# transfering dataset
df_dataset = fs.transfer_dataset(df_tripdata, df_polygon, period_start, period_end, origin_time)

print()
fs.show_data_situation(df_dataset)
print()

fileName = period_start.strftime("%Y%m%d")+"_"+period_start.strftime("%H")+"-"+str(int((period_end-period_start).seconds/60))+"m"

# write data
fs.printData(fileName, df_dataset, 10, 6, 0, 7200)