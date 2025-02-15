#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --time=2:15:00
#SBATCH --array=1-2
#SBATCH --output=/dev/null

module load eigen
module load gcc

i=1

# Define parameters
param_dir="Param_mode_1a.txt"
vehicles="manhattan-vehicles"
modes=(2)
algorithms=(6)
directory="Instances-120"
instances=("20150926_07-120m" "20160109_07-120m")
num_vehicles_list=(1500)

for mode in "${modes[@]}"; do
  for algorithm in "${algorithms[@]}"; do
    for instance in "${instances[@]}"; do
      for num_vehicles in "${num_vehicles_list[@]}"; do
        if [ "$SLURM_ARRAY_TASK_ID" -eq "$i" ]; then
          bin/realtime_DARP "$vehicles" "$directory" "$instance" "$num_vehicles" "$algorithm" "$mode" "$param_dir" 1
          # bin/realtime_DARP "$directory" "$instance" "$num_vehicles" "$algorithm" "$mode" > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
        fi
        ((i++))
      done
    done
  done
done

sleep 60
