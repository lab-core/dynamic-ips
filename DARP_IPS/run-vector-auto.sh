#!/bin/bash
#SBATCH --mem=18G
#SBATCH --cpus-per-task=16
#SBATCH --time=2:15:00
#SBATCH --array=1-12
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
param_files[1]="Ab_drop Ab_truncate"  # Mode 1 has two parameter files
param_files[2]="ACG-LP ACG-CP ACG-AUXP"  # Mode 2 has three parameter files

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))
instances=(
  "20150706_07-120m"
  "20150828_07-120m"
  "20151025_07-120m"
  "20160109_07-120m"
  "20160316_07-120m"
  "20160401_07-120m"
)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 1; do
  algorithm=${algorithms[$mode]}  # Select algorithm for the current mode
#  for instance_path in "${INSTANCES[@]}"; do
#    instance=$(basename "$instance_path")
  for instance in "${instances[@]}"; do
    for param_dir in ${param_files[$mode]}; do
    jobs[$i]="$vehicles_1 $directory $instance 2000 $algorithm $mode $param_dir 1"
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
