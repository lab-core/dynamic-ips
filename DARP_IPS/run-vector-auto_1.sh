#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=24G
#SBATCH --time=2:40:00
#SBATCH --array=1-9
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Define fixed parameters
vehicles="sufficient_manhattan-vehicles-300"
directory="Instances_12-14"
algorithm=6

# Define parameter files for each mode
declare -A param_files
param_files[1]="Param_mode_1.txt"
param_files[2]="Param_mode_2.txt"
param_files[3]="Param_mode_3.txt"

# Define instance groups and corresponding vehicle counts
instances_1000=("20150828_12-120m" "20151130_12-120m" "20160222_12-120m" "20151230_12-120m")
instances_1100=("20160316_12-120m" "20160512_12-120m")
instances_1400=("20160521_12-120m" "20151025_12-120m" "20150926_12-120m")

# Create a single array containing all instance-mode combinations
declare -a jobs
i=1

for mode in 2 1; do
  param_dir=${param_files[$mode]}  # Select the correct parameter file

  for instance in "${instances_1000[@]}"; do
    jobs[$i]="$vehicles $directory $instance 1000 $algorithm $mode $param_dir"
    ((i++))
  done
  for instance in "${instances_1100[@]}"; do
    jobs[$i]="$vehicles $directory $instance 1100 $algorithm $mode $param_dir"
    ((i++))
  done
  for instance in "${instances_1400[@]}"; do
    jobs[$i]="$vehicles $directory $instance 1400 $algorithm $mode $param_dir"
    ((i++))
  done
done

# Execute the assigned task based on SLURM_ARRAY_TASK_ID
if [[ -n "${jobs[$SLURM_ARRAY_TASK_ID]}" ]]; then
  bin/realtime_DARP ${jobs[$SLURM_ARRAY_TASK_ID]}
fi

# Add a delay to avoid overloading the system
sleep 60
