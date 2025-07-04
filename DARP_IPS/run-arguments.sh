#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=64G
#SBATCH --time=15:15:00
#SBATCH --array=1-9
#SBATCH --output=/dev/null

# Load required modules
module load eigen gcc

# Define fixed parameters
vehicle_folder="vehicles_byDemand"
inst_folder="Instances_2h-12"
param_dir="BatchParameters"

# Define algorithms for each mode
declare -A algorithms
algorithms[1]="2"  # Mode 1 -> Algorithm 2
algorithms[2]="6 3"  # Mode 2 -> Algorithm 6

# Define parameter files for each mode
declare -A param_files
scenario_files[1]="exact"  # Mode 1 has two parameter files
scenario_files[2]="ACG-LP"  # Mode 2 has three parameter files

# Define instance groups and corresponding vehicle counts
instances_1000=("20150828_12-120m" "20151130_12-120m" "20160222_12-120m" "20151230_12-120m")
instances_1100=("20160316_12-120m" "20160512_12-120m")
instances_1400=("20160521_12-120m" "20151025_12-120m" "20150926_12-120m")

# Create a single array containing all instance-mode combinations
declare -a jobs
i=1

for mode in 1; do
  for algorithm in ${algorithms[$mode]}; do  # Select algorithm for the current mode
    for scenario in ${scenario_files[$mode]}; do  # Iterate over multiple parameter files for each mode

      for instance in "${instances_1000[@]}"; do
        jobs[$i]="--vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $instance --num-vehicles 1000 --main-algo $algorithm --sol-mode $mode --paramfile $param_dir --scenario $scenario --save-scratch 1"
        ((i++))
      done
      for instance in "${instances_1100[@]}"; do
        jobs[$i]="--vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $instance --num-vehicles 1100 --main-algo $algorithm --sol-mode $mode --paramfile $param_dir --scenario $scenario --save-scratch 1"
        ((i++))
      done
      for instance in "${instances_1400[@]}"; do
        jobs[$i]="--vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $instance --num-vehicles 1400 --main-algo $algorithm --sol-mode $mode --paramfile $param_dir --scenario $scenario --save-scratch 1"
        ((i++))
      done
    done
  done
done

# Execute the assigned task based on SLURM_ARRAY_TASK_ID
if [[ -n "${jobs[$SLURM_ARRAY_TASK_ID]}" ]]; then
  bin/realtime_DARP ${jobs[$SLURM_ARRAY_TASK_ID]}
fi

# Add a delay to avoid overloading the system
sleep 60