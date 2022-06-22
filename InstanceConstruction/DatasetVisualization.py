# -*- coding: utf-8 -*-
"""
Created on Tue May 24 12:35:43 2022

@author: Ella
"""


import functions as fs
import pandas as pd
import datetime as dt


# Importing trip record data and polygon points of desired area

trip_filename = "yellow_tripdata_2016-06.csv"
polygon_filename = "Manhattan_polygon_neigh1-6.csv"

# Selecting the operating system is just for setting the resolution
# It can be set on windows or osx
os = "osx"

df_tripdata = pd.read_csv(trip_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)

# determine the desired period of time for trip requests
year = 2016
month = 6
day = 14
period_start = dt.datetime(year,month,day,7,0,0)
period_end = dt.datetime(year,month,day,23,0,0)
origin_time = dt.datetime(year,month,day,0,0,0)


df_dataset = fs.clean_dataset(df_tripdata, period_start, period_end)
fs.showRequests_PerHour(df_dataset, df_polygon, period_start, os)

