#!/bin/bash

DIRECTORY="Instances-120_Epoch"
MAIN_DIR="datasets/Instances-120_Epoch"

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find ./$MAIN_DIR -mindepth 1 -maxdepth 1 -type d -print | sort))

i=1
for vehicles in manhattan-vehicles
do
  for mode in 1
  do
    for algorithm in 2
    do
      for instance_path in "${INSTANCES[@]}"; 
      do
        instance=$(basename "$instance_path")
        for num_vehicles in 2000
        do
          echo "bin/realtime_DARP $vehicles $DIRECTORY $instance $num_vehicles $algorithm $mode"
        done
      done
    done
  done
done
