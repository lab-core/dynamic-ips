#!/bin/bash

# Define constants for command parameters
program="bin/realtime_DARP"
map_file="sufficient_manhattan-vehicles-300"
instance_dir="Instances_12-14_Last"
args="1500 6 1"
param_dir="Parameters.txt"

DIRECTORY="Instances_12-14_Last"
MAIN_DIR="datasets/$DIRECTORY"

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$MAIN_DIR" -mindepth 1 -maxdepth 1 -type d -print | sort))

for instance in "${INSTANCES[@]}"; do
  instance_basename=$(basename "$instance")
  "$program" "$map_file" "$DIRECTORY" "$instance_basename" $args "$param_dir"
done