
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 21:25:25 2022
this file prepare and save the row dataset

@author: Ella
"""

import functions as fs
import pandas as pd
import datetime as dt
import os


# Importing trip record data and polygon points of desired area

trip_filename = "yellow_tripdata_2016-06.csv"
polygon_filename = "Manhattan_polygon_neigh1-3.csv"
capacity = 6
df_tripdata = pd.read_csv(trip_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)

# determine the desired period of time for trip requests
year = 2016
month = 6
day = 3
period_start = dt.datetime(year,month,day,10,0,0)
period_end = dt.datetime(year,month,day,14,0,0)
origin_time = dt.datetime(year,month,day,0,0,0)


# transfering dataset
df_dataset = fs.transfer_dataset(df_tripdata, df_polygon, period_start, period_end, origin_time)
df_dataset = fs.limit_capacity(df_dataset, capacity)


fs.show_data_situation(df_tripdata)

fileName = period_start.strftime("%Y%m%d")+"_"+period_start.strftime("%H")+"-"+str(int((period_end-period_start).seconds/60))+"m"
if not os.path.exists(fileName):
    os.mkdir(fileName)

csvFile = fileName + "/" + "Dataset_" + fileName + ".csv"
df_dataset.to_csv(csvFile, index=False)