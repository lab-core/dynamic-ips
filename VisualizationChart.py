# -*- coding: utf-8 -*-
"""
Created on Tue Jan  4 19:05:28 2022

@author: Ella
"""

#import matplotlib; matplotlib.use("TkAgg")
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Defining the update function
def update_allData(frame_number):
    data = np.array(tripdata_data[tripdata_data[:,6] <= frame_number])
    pick_data = np.array(data[:,2:4])
    drop_data = np.array(data[:,3:5])

    scats[0].set_offsets(pick_data)
    scats[1].set_offsets(drop_data)
    
    for v in range(nbVehicle):
        vehicle_data = results[v][results[v][:,2] <= frame_number]
        routes[v].set_data(vehicle_data[:,4], vehicle_data[:,5])       
    return scats, routes

# only show a part of nodes in the frame
def update1(frame_number):
    data = tripdata_data[tripdata_data[:,6] <= frame_number]
    data = data[data[:,8] >= frame_number - 2 * frame_length]
    pick_data = np.array(data[:,2:4])
    drop_data = np.array(data[:,4:6])

    scats[0].set_offsets(pick_data)
    scats[1].set_offsets(drop_data)
    
    for v in range(nbVehicle):
        vehicle_data = results[v][results[v][:,2] <= frame_number]
        vehicle_data = vehicle_data[vehicle_data[:,2] >= frame_number - 6 * frame_length]
        routes[v].set_data(vehicle_data[:,4], vehicle_data[:,5])       
    return scats, routes
    
# show completed nodes in gray color 
def update2(frame_number):
    data = tripdata_data[tripdata_data[:,6] <= frame_number]
    incomplete_data = data[data[:,8] >= frame_number - frame_length]
    complete_data = data[data[:,8] < frame_number - frame_length]
    pick_data = incomplete_data[:,2:4]
    drop_data = incomplete_data[:,4:6]
    complete_data = np.concatenate([complete_data[:,2:4], complete_data[:,4:6]])

    scats[0].set_offsets(pick_data)
    scats[1].set_offsets(drop_data)
    scats[2].set_offsets(complete_data)
    
    for v in range(nbVehicle):
        vehicle_data = results[v][results[v][:,2] <= frame_number]
        vehicle_data = vehicle_data[vehicle_data[:,2] >= frame_number - 6 * frame_length]
        routes[v].set_data(vehicle_data[:,4], vehicle_data[:,5])       
    return scats, routes


# show completed nodes in gray color (separate pick up and drop off)
def update3(frame_number):
    # update route data
    for v in range(nbVehicle):
        vehicle_data = results[v][results[v][:,2] <= frame_number]
        completed_nodes = vehicle_data[vehicle_data[:,2] < frame_number - 3 * frame_length]
        if np.shape(completed_nodes)[0] >= 1:
            current_nodes = vehicle_data[np.shape(completed_nodes)[0]-1:,:]
        else:
            current_nodes = vehicle_data[np.shape(completed_nodes)[0]:,:]
#        completed_nodes = vehicle_data[:np.shape(current_nodes)[0]+1,:]
        comp_routes[v].set_data(completed_nodes[:,4], completed_nodes[:,5])
        routes[v].set_data(current_nodes[:,4], current_nodes[:,5]) 
    
    # update scatter data 
    data = tripdata_data[tripdata_data[:,6] <= frame_number]  
    # pick up data
    pick_data = data[data[:,7] >= frame_number - frame_length]
    pick_data = pick_data[:,2:4]
    
    # drop off data
    drop_data = data[data[:,8] >= frame_number - frame_length]
    drop_data = drop_data[:,4:6]
    
    # complete data
    complete_pick = data[data[:,7] < frame_number - frame_length]
    complete_drop = data[data[:,8] < frame_number - frame_length]
    
    complete_data = np.concatenate([complete_pick[:,2:4], complete_drop[:,4:6]])

    scats[0].set_offsets(pick_data)
    scats[1].set_offsets(drop_data)
    scats[2].set_offsets(complete_data)
    
    return scats, routes


# Reading data

frame_length = 10
instance_Name = "20150713_16-03m"
datapath = "DARP_IPS/datasets/" 
nbVehicle = 4

polygon_filename = "Manhattan_polygon_neigh1-6.csv"
requests_filename = datapath + instance_Name + "/Requests_" + instance_Name + ".csv"
routes_filename = datapath + instance_Name + "/Routes_" + instance_Name + ".csv"

df_tripdata = pd.read_csv(requests_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)
df_routes = pd.read_csv(routes_filename, sep=',', header = 0)

polygon_data =df_polygon.to_numpy(dtype='double')
tripdata_data =df_tripdata.to_numpy(dtype='double')
result_data = df_routes.drop(columns =['NodeID']).to_numpy(dtype='double')
results = []
for v in range(nbVehicle):
    results.append(np.array(result_data[result_data[:,0] == v]))
    
# Visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(autoscale_on=False)
ax.set_aspect('equal')
#ax.grid()
ax.set_xlim(round(min(polygon_data[:,0]),2)-0.01, round(max(polygon_data[:,0]),2)+0.01)
ax.set_ylim(round(min(polygon_data[:,1]),2)-0.01, round(max(polygon_data[:,1]),2)+0.01)
ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
line, = ax.plot(np.array(polygon_data[:,0]), np.array(polygon_data[:,1]) , 'grey')
source = ax.scatter([results[0][0][4]], [results[0][0][5]], c = "grey", s = 110, marker = ",")
results[0]

comp_routes = [ax.plot([], [], c = 'silver', linewidth=0.5)[0] for _ in results]
routes = [ax.plot([], [], marker=".", markersize=10)[0] for _ in results]
#scats = [ax.scatter([], [], c = "g", s = 70, alpha=0.5), ax.scatter([], [], c = "y", s = 70, alpha=0.5)]
scats = [ax.scatter([], [], c = "g", s = 70, alpha=0.7, label="pickup"), ax.scatter([], [], c = "y", s = 70, alpha=0.7, label="dropoff"), 
         ax.scatter([], [], c = "lightgrey", s = 70, alpha=0.5, label="completed")]
ax.legend()

ani = FuncAnimation(fig, func = update3, frames = np.arange(0,800,frame_length), interval = 1000, repeat = False)
plt.show()