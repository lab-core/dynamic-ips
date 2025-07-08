#!/bin/bash
#SBATCH --mem=20G
#SBATCH --cpus-per-task=16
#SBATCH --time=2:15:00
#SBATCH --array=1-24
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Define fixed parameters
vehicles_1="vehicles_uniform"
vehicles_2="vehicles_byDemand"
directory="Instances_2h-7"
main_dir="datasets/$directory"
param_dir="BatchParameters"

# Define algorithms for each mode
declare -A algorithms
algorithms[1]="2"  # Mode 1 -> Algorithm 2
algorithms[2]="6"  # Mode 2 -> Algorithm 6

# Define scenario for each mode
declare -A scenario_files
scenario_files[1]="commit"
scenario_files[2]="ACG-LP"

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find "./$main_dir" -mindepth 1 -maxdepth 1 -type d -print | sort))
instances=(
  "20150828_07-960m"
  "20150926_07-960m"
  "20150917_07-960m"
)

vehicle_counts=(2000)

# Create a single array containing all instance-mode-parameter combinations
declare -a jobs
i=1

for mode in 1; do
  for algorithm in ${algorithms[$mode]}; do  # Select algorithm for the current mode
    for vehicle_count in "${vehicle_counts[@]}"; do
      for instance_path in "${INSTANCES[@]}"; do
        instance=$(basename "$instance_path")
#      for instance in "${instances[@]}"; do
        for scenario in ${scenario_files[$mode]}; do
        jobs[$i]="--vehicle-folder $vehicles_1 --inst-folder $directory --instance-name $instance --num-vehicles $vehicle_count --main-algo $algorithm --sol-mode $mode --paramfile $param_dir --scenario $scenario --save-scratch 1"
        ((i++))
        done
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
