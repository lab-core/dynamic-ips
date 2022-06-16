# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 08:54:05 2022

@author: Ella
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from string import Template
import itertools


def on_press(event):
    if event.key.isspace():
        if ani.running:
            ani.event_source.stop()
        else:
            ani.event_source.start()
        ani.running ^= True
        
def color_generator(size):
    rand_color = []
    for j in range(size):
        rand_color.append("#"+''.join([random.choice('ABCDEF0123456789') for i in range(6)]))
    return rand_color


# def data_gen():
#     for i in range(np.shape(frame_data)[0]):
#         print(frame_data[i,:])
#         epoch_num = frame_data[i,0]
#         iter_num = frame_data[i,1]
    
#    yield epoch_num, iter_num
        
def update(counter):
    epoch_number =  frame_data[counter,0]
    iter_number = frame_data[counter,1]
    # update route data
    for v in range(nbVehicle):
        epoch_nodes = currResults[v][currResults[v][:,0] == epoch_number]
        
        iter_IDs = epoch_nodes[epoch_nodes[:,1] == iter_number]
        iter_nodes = np.array([locations[index] for index in (iter_IDs[:,6]).astype(int)])


        completed_IDs = compResults[v][compResults[v][:,0] == epoch_number]
        completed_nodes = np.array([locations[index] for index in (completed_IDs[:,5]).astype(int)])
        
        comp_routes[v].set_data(completed_nodes[:,1], completed_nodes[:,2])
        curr_routes[v].set_data(iter_nodes[:,1], iter_nodes[:,2])
    
    # update scatter data 
    data = request_data[request_data[:,4] <= simulStart + epoch_number * EpochLength]
    available = data[data[:,5] > simulStart + (epoch_number+1) * EpochLength]
    completed = data[data[:,6] <= simulStart + (epoch_number+1) * EpochLength]
    onboard = data[data[:,5] <= simulStart + (epoch_number+1) * EpochLength]
    onboard = onboard[onboard[:,6] > simulStart + (epoch_number+1) * EpochLength]
    
    
    for v in range(nbVehicle):
        vehicle_onboard = onboard[onboard[:,7] == v]
        planned = onboard[onboard[:,7] != v]
        remained = np.concatenate([available, vehicle_onboard])
        remained_IDs = np.concatenate([available, vehicle_onboard])
        remained_pick = np.array([locations[index] for index in (remained_IDs[:,2]).astype(int)])
        remained_drop = np.array([locations[index] for index in (remained_IDs[:,3]).astype(int)])
        
        unavailable = np.concatenate([completed, planned])
        unavailable_IDs = np.concatenate([completed, planned])
        unavailable_pick = np.array([locations[index] for index in (unavailable_IDs[:,2]).astype(int)])
        unavailable_drop = np.array([locations[index] for index in (unavailable_IDs[:,3]).astype(int)])
        if np.shape(unavailable_pick) == (0,):
            unavailable_pick = np.empty([0,2], dtype= 'float64')
            
        if np.shape(unavailable_drop) == (0,):
            unavailable_drop = np.empty([0,2], dtype= 'float64')
        
        if np.shape(remained_pick) == (0,):
            remained_pick = np.empty([0,2], dtype= 'float64')
        
        if np.shape(remained_drop) == (0,):
            remained_drop = np.empty([0,2], dtype= 'float64')
        
       
        
        scats[v][0].set_offsets(unavailable_pick[:,1:3])
        scats[v][1].set_offsets(unavailable_drop[:,1:3])
        scats[v][2].set_offsets(remained_pick[:,1:3])
        scats[v][3].set_offsets(remained_drop[:,1:3])
        
        
        scats[v][0].set_edgecolors([point_colors[index] for index in (unavailable[:,0]).astype(int)])
        scats[v][1].set_edgecolors([point_colors[index] for index in (unavailable[:,0]).astype(int)])
        scats[v][2].set_facecolors([point_colors[index] for index in (remained[:,0]).astype(int)])
        scats[v][3].set_facecolors([point_colors[index] for index in (remained[:,0]).astype(int)])

        

    
    title.set_text(title_template.substitute(starttime= str((epoch_number)*EpochLength), 
                                             endtime= str((epoch_number+1)*EpochLength), 
                                             epochs= str(epoch_number), 
                                             iters = str(iter_number)))
    return scats



########################################################################
#                       M A I N   P R O G R A M                        #
########################################################################

frame_length = 1
instance_Name = "20150716_12-60m"
results_date = "20220517-1131"
datapath = "DARP_IPS/datasets/" 
nbVehicle = 6
speedInterval = 1500
EpochLength = 30
simulStart = 43200

polygon_filename = "Manhattan_polygon_neigh1-6.csv"
requests_filename = datapath + instance_Name + "/Results_" + results_date + "/Requests_" + instance_Name + ".csv"
compRoutes_filename = datapath + instance_Name+ "/Results_" + results_date + "/finalSolution_" + instance_Name + ".csv"
currRoutes_filename = datapath + instance_Name + "/Results_" + results_date + "/epochSolution_" + instance_Name + ".csv"
locations_filename = datapath + instance_Name + "/LOCATION_" + instance_Name + ".csv"

#######################   Reading data  ###############################
df_requests = pd.read_csv(requests_filename, sep=',', header = 0)
df_polygon = pd.read_csv(polygon_filename, sep=',', header = 0)
df_currRoutes = pd.read_csv(currRoutes_filename, sep=',', header = 0)
df_compRoutes = pd.read_csv(compRoutes_filename, sep=',', header = 0)
df_locations = pd.read_csv(locations_filename, sep=',', header = 0)

####################### Preprocess data  ##############################
polygon_data =df_polygon.to_numpy(dtype='double')
request_data =df_requests.to_numpy(dtype='double')
comp_data = df_compRoutes.drop(columns =['NodeID']).to_numpy(dtype='double')
curr_data = df_currRoutes.drop(columns =['NodeID']).to_numpy(dtype='double')
frame_data = df_currRoutes[['Epoch', ' ISUDIter']].drop_duplicates().to_numpy(dtype='int')
locations = df_locations.to_numpy(dtype='double')

currResults = []
for v in range(nbVehicle):
    currResults.append(np.array(curr_data[curr_data[:,2] == v]))

compResults = []
for v in range(nbVehicle):
    compResults.append(np.array(comp_data[comp_data[:,1] == v]))
    
#######################  define colors   ##############################
point_colors = color_generator(np.shape(request_data)[0])
colorList = ['darkred','seagreen','steelblue', 'darkorange', 'deeppink','darkgoldenrod']
route_color = itertools.cycle(colorList)
complete_color = itertools.cycle(colorList)

#######################   define plots   ##############################
fig = plt.figure()
gs = fig.add_gridspec(2, 3, hspace=0, wspace=0)
axs = gs.subplots(sharex='col', sharey='row')
fig.tight_layout()

title_template = Template('time = ${starttime} - ${endtime} (s)       solution epoch = ${epochs}       ISUD iteration = ${iters}')
title = plt.suptitle(t='', fontsize = 30, fontname="Garamond", fontweight="bold")

scats = []
comp_routes = []
curr_routes = []
v_counter = 0

for ax in axs.ravel():

    ax.set_xlim(round(min(polygon_data[:,0]),2)-0.01, round(max(polygon_data[:,0]),2)+0.01)
    ax.set_ylim(round(min(polygon_data[:,1]),2)-0.01, round(max(polygon_data[:,1]),2)+0.01)
    ax.set_xlabel('Latitude', fontsize=20)
    ax.set_ylabel('Longitude', fontsize=20)
    line, = ax.plot(np.array(polygon_data[:,0]), np.array(polygon_data[:,1]) , 'grey')
    source = ax.scatter([locations[compResults[v_counter][0][5].astype(int)][1]], 
                        [locations[compResults[v_counter][0][5].astype(int)][2]], c = "grey", s = 150, marker = ",")

    scats.append([ax.scatter([], [], c= 'w',  s = 60, alpha=1, linewidth = 1.5, edgecolor='y', label="unavailable pickup"),
                  ax.scatter([], [], c= 'w', s = 60, marker = ',', linewidth = 1.5, edgecolor='y', alpha=1, label="unavailable dropoff"),
                  ax.scatter([], [], c= 'c', s = 60, alpha=1, label="pickup"),
                  ax.scatter([], [], c= 'c', s = 60, marker = ',', alpha=1, label="dropoff")])
    ax.legend(prop={"size":20})
    
    comp_routes.append(ax.plot([], [], linewidth=1, c = next(complete_color))[0])
    
    curr_routes.append(ax.plot([], [], linewidth=3, c = next(route_color))[0])
    v_counter = v_counter + 1

    
#######################   animation function   ##############################
fig.canvas.mpl_connect('key_press_event', on_press)
ani = animation.FuncAnimation(fig, func = update, frames = np.arange(0,np.shape(frame_data)[0],frame_length), interval = speedInterval, repeat = True)

ani.running = True

figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()

plt.show()