#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --time=2:15:00
#SBATCH --array=1-48
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Build the project
cmake --build cmake-build-release --target all

# Define fixed parameters
vehicles="manhattan-vehicles"
directory="Instances-120"
main_dir="datasets/$directory"

# Define parameter files for each mode
declare -A param_files
param_files[1]="Param_mode_1a.txt Param_mode_1b.txt"  # Mode 1 has two parameter files
param_files[2]="Param_mode_2a.txt Param_mode_2b.txt Param_mode_2c.txt"  # Mode 2 has three parameter files

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))

# Define vehicle counts
num_vehicles_list=(2000)
# Uncomment to use multiple vehicle counts
# num_vehicles_list=(1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 1 2; do
  for param_dir in ${param_files[$mode]}; do  # Iterate over multiple parameter files for each mode
    for instance_path in "${INSTANCES[@]}"; do
      instance=$(basename "$instance_path")

      for num_vehicles in "${num_vehicles_list[@]}"; do
        jobs[$i]="$vehicles $directory $instance $num_vehicles 6 $mode $param_dir"
        ((i++))
      done
    done
  done
done

# Execute the assigned task based on SLURM_ARRAY_TASK_ID
if [[ -n "${jobs[$SLURM_ARRAY_TASK_ID]}" ]]; then
  bin/realtime_DARP ${jobs[$SLURM_ARRAY_TASK_ID]}
  # Uncomment for output logging
  # bin/realtime_DARP ${jobs[$SLURM_ARRAY_TASK_ID]} > "/scratch/amirelah/dynamic-ips/${jobs[$SLURM_ARRAY_TASK_ID]}.txt" 2>&1
fi

# Add a delay to avoid overloading the system
sleep 60
