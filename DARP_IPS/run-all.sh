#!/bin/bash

# Define fixed parameters
vehicles_1="vehicles_uniform"
vehicles_2="vehicles_byDemand"
directory="Instances_30s"
main_dir="datasets/$directory"
param_dir="AnyParameters"

# Define algorithms for each mode
declare -A algorithms
algorithms[1]="6"
algorithms[2]="6"

# Define scenario for each mode
declare -A scenario_files
scenario_files_1=("Dual_1_0" "Dual_1_1" "Dual_7_0" "Dual_7_1" "Dual_3_0" "Dual_3_1")
scenario_files_2=("Dual_1_0" "Dual_1_1" "Dual_7_0" "Dual_7_1")

# Dynamically create the INSTANCES array
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))
instances=("20150828_12-120m")

vehicle_counts=(1000)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 1; do
  for algorithm in ${algorithms[$mode]}; do
    for vehicle_count in "${vehicle_counts[@]}"; do
      for instance_path in "${INSTANCES[@]}"; do
        instance=$(basename "$instance_path")
        for scenario in "${scenario_files_1[@]}"; do
          jobs[$i]="--vehicle-folder $vehicles_1 --inst-folder $directory --instance-name $instance --num-vehicles $vehicle_count --main-algo $algorithm --sol-mode $mode --paramfile $param_dir --scenario $scenario --save-scratch 0 --initial-state 2"
          ((i++))
        done
      done
    done
  done
done

# Run all jobs sequentially
for idx in "${!jobs[@]}"; do
  echo "Running job $idx: ${jobs[$idx]}"
  bin/realtime_DARP ${jobs[$idx]}
done
