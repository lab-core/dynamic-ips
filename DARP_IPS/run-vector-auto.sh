#!/bin/bash
#SBATCH --time=5:30:00
#SBATCH --mem=64G
#SBATCH --cpus-per-task=16
#SBATCH --array=1-48
#SBATCH --output=/dev/null
#SBATCH --mail-type=END
#SBATCH --mail-user=elahe.amiri66@gmail.com


module load eigen
module load gcc
cmake --build cmake-build-release --target all

DIRECTORY="Instances_5h"
MAIN_DIR="datasets/Instances_5h"

# Dynamically create the INSTANCES array with paths to each test subdirectory
INSTANCES=($(find ./$MAIN_DIR -mindepth 1 -maxdepth 1 -type d -print | sort))

i=1
for vehicles in sufficient_manhattan-vehicles-300
do
  for mode in 2
  do
    for algorithm in 3
    do
      for instance_path in "${INSTANCES[@]}";
      do
        instance=$(basename "$instance_path")
        for num_vehicles in 1500
#        for num_vehicles in 1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500
        do
          if [ $SLURM_ARRAY_TASK_ID -eq $i ]
            then
              bin/realtime_DARP $vehicles $DIRECTORY $instance $num_vehicles $algorithm $mode
#             bin/realtime_DARP $vehicles $DIRECTORY $instance $num_vehicles $algorithm $mode > "/scratch/amirelah/dynamic-ips/${vehicles}_${DIRECTORY}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
            fi
            (( i = $i + 1 ))
        done
      done
    done
  done
done

sleep 60