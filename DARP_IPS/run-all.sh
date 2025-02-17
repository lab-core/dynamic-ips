#!/bin/bash

# Define constants for command parameters
program="bin/realtime_DARP"
map_file="sufficient_manhattan-vehicles-300"
instance_dir="Instances_12-14_Last"
args="1500 6 1"
param_dir="Parameters.txt"

DIRECTORY="Instances_12-14_Last"
MAIN_DIR="datasets/$DIRECTORY"

# List of instances based on the previous grouping approach
instances_1000=("20150828_12-120m" "20151130_12-120m" "20160222_12-120m" "20151230_12-120m")
instances_1100=("20160316_12-120m" "20160512_12-120m")
instances_1400=("20160521_12-120m" "20151025_12-120m" "20150926_12-120m")

# Define mode and corresponding algorithm
mode=2
algorithm=6

# Define parameter files for mode 2
param_files=("Param_mode_2a.txt" "Param_mode_2b.txt" "Param_mode_2c.txt" "Param_mode_2d.txt")

# Run the program for each instance and parameter file
for param_file in "${param_files[@]}"; do
  for instance in "${instances_1000[@]}"; do
    "$program" "$map_file" "$DIRECTORY" "$instance" 1000 "$algorithm" "$mode" "$param_file" 0
  done
  for instance in "${instances_1100[@]}"; do
    "$program" "$map_file" "$DIRECTORY" "$instance" 1100 "$algorithm" "$mode" "$param_file" 0
  done
  for instance in "${instances_1400[@]}"; do
    "$program" "$map_file" "$DIRECTORY" "$instance" 1400 "$algorithm" "$mode" "$param_file" 0
  done
done

# Add a delay to avoid overloading the system
sleep 60