#!/bin/bash
#SBATCH --mem=100G
#SBATCH --cpus-per-task=24
#SBATCH --time=6:35:00
#SBATCH --array=1-1
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Define fixed parameters
vehicles_1="manhattan-vehicles"
vehicles_2="sufficient_manhattan-vehicles-300"
directory="Instances_12-14_New"
main_dir="datasets/$directory"

# Define algorithms for each mode
declare -A algorithms
algorithms[1]=2  # Mode 1 -> Algorithm 2

# Define parameter files for each mode
declare -A param_files
param_files[1]="nbPick4"  # Mode 1 has two parameter files

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))
instances=("20150828_12-120m_59")

# Define vehicle counts
num_vehicles_list=(1500)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 1; do
  algorithm=${algorithms[$mode]}  # Select algorithm for the current mode
  for param_dir in ${param_files[$mode]}; do
#    for instance_path in "${INSTANCES[@]}"; do
#      instance=$(basename "$instance_path")
    for instance in "${instances[@]}"; do
      jobs[$i]="$vehicles_2 $directory $instance 1500 $algorithm $mode $param_dir 1"
      ((i++))
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
