# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 21:25:25 2022

@author: Ella
"""

import functions as fs
import pandas as pd
import datetime as dt
import os


# Importing trip record data and polygon points of desired area
nbVehices = 25
capacity = 6

FolderName = "20150706_12-240m"
dataset_fileName = FolderName + "/Dataset_" + FolderName + ".csv"
df_initial_dataset = pd.read_csv(dataset_fileName, sep=',', header = 0)

# determine the desired period of time for trip requests
year = 2015
month = 7
day = 6
period_start = dt.datetime(year,month,day,12,0,0)
period_end = dt.datetime(year,month,day,16,0,0)
origin_time = dt.datetime(year,month,day,0,0,0)

start_seconds = (period_start - origin_time).total_seconds()
end_seconds = (period_end - origin_time).total_seconds()

df_dataset = fs.limit_time(df_initial_dataset, period_start, period_end, origin_time)
fs.show_data_situation(df_dataset)

df_locations = fs.list_locations(df_dataset)
fs.show_data_situation(df_locations)

print()

fileName = period_start.strftime("%Y%m%d")+"_"+period_start.strftime("%H")+"-"+str(int((period_end-period_start).seconds/60))+"m"
fs.saveInstance(fileName, df_dataset, df_locations, nbVehices, capacity, start_seconds, end_seconds*2)

#query duration data
df_duration_data = fs.calculate_durations(df_locations)
fs.printDurations(fileName, df_duration_data)