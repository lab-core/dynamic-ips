#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --time=2:15:00
#SBATCH --array=1-48
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Define fixed parameters
vehicles_1="manhattan-vehicles"
vehicles_2="sufficient_manhattan-vehicles-300"
directory="Instances-120"
main_dir="datasets/$directory"

# Define algorithms for each mode
declare -A algorithms
algorithms[1]=2  # Mode 1 -> Algorithm 2
algorithms[2]=6  # Mode 2 -> Algorithm 6

# Define parameter files for each mode
declare -A param_files
param_files[1]="Param_mode_1a.txt"  # Mode 1 has two parameter files
param_files[2]="Param_mode_2c.txt"  # Mode 2 has three parameter files
param_files[3]="Param_mode_1r.txt"
param_files[4]="Param_mode_2r.txt"

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))

# Define vehicle counts
num_vehicles_list=(1500)
# Uncomment to use multiple vehicle counts
# num_vehicles_list=(1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 2; do
  algorithm=${algorithms[$mode]}  # Select algorithm for the current mode
  for instance_path in "${INSTANCES[@]}"; do
    instance=$(basename "$instance_path")

    param_dir=${param_files[$mode]}
    jobs[$i]="$vehicles_2 $directory $instance 1500 $algorithm $mode $param_dir 1"
    ((i++))

    param_dir=${param_files[$((mode+2))]}
    jobs[$i]="$vehicles_1 $directory $instance 2000 $algorithm $mode $param_dir 1"
    ((i++))
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
