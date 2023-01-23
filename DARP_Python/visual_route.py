import os
import os.path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from string import Template
import itertools
from Dataset import *
import visualize as vf


########################################################################
#                       M A I N   P R O G R A M                        #
########################################################################
parent_path = os.path.dirname(os.getcwd())
frame_length = 1
instance_Name = "20160225_07-120m"
results_date = "DYNAMIC_GREEDY_20230120-0342"
data_path = parent_path + "/DARP_IPS/datasets/Instances-120/"
nbVehicle = 2000
speedInterval = 1500
epoch_length = 30
simulation_start = 25200

manhattan_districts = Network()
manhattan_districts.read_network_data('manhattan-geo', location_to_cell_file='location_to_cell',
                                      edge_time_matrix_file='edge_time_matrix', stop_matrix_file='stop_matrix')

requests_filename = data_path + instance_Name + "/" + results_date + "/Requests_" + instance_Name + ".csv"
compRoutes_filename = data_path + instance_Name + "/" + results_date + "/Routes_" + instance_Name + ".csv"

#######################   Reading data  ###############################
df_requests = pd.read_csv(requests_filename, sep=',', header=0)
df_compRoutes = pd.read_csv(compRoutes_filename, sep=',', header=0)

####################### Preprocess data  ##############################
request_data = df_requests.to_numpy(dtype='double')
comp_data = df_compRoutes.drop(columns=['NodeID']).to_numpy(dtype='double')
compResults = []
for v in range(nbVehicle):
    compResults.append(np.array(comp_data[comp_data[:, 0] == v]))

#######################   define plots   ##############################
fig, ax = vf.plot_districts(district_network=manhattan_districts, print_id=False)

pick_points = np.array([manhattan_districts.locations[index] for index in (request_data[:,2]).astype(int)])
drop_points = np.array([manhattan_districts.locations[index] for index in (request_data[:,3]).astype(int)])
#plt.scatter(pick_points[:, 1], pick_points[:, 2], c='green', s=3,  alpha=1, label="pickup")
#plt.scatter(drop_points[:, 1], drop_points[:, 2], c='blue', s=5,  alpha=0.3, label="dropOff", marker = ",")
for v in range(nbVehicle):
    if (len(comp_data[comp_data[:, 0] == v]) == 1):
        route_nodes = np.array([manhattan_districts.locations[index] for index in (compResults[v][:, 4]).astype(int)])
        plt.plot(route_nodes[:,1], route_nodes[:,2], marker = "s", c='red', markersize=2)

fig.savefig(data_path + "unused.png")
fig.show()

