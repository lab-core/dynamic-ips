#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=80G
#SBATCH --time=02:15:00
#SBATCH --array=1-3
#SBATCH --output=/dev/null


module load eigen
module load gcc

i=1
for algorithm in 2
do
  for directory in Instances-120
  do
    for instance in 20150715_07-120m 20150804_07-120m 20160613_07-120m
    do
      for num_vehicles in 2000
#      for num_vehicles in 1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500
      do
        if [ $SLURM_ARRAY_TASK_ID -eq $i ]
        then
#          bin/realtime_DARP $directory $instance $num_vehicles $algorithm
          bin/realtime_DARP $directory $instance $num_vehicles $algorithm > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}.txt" 2>&1
        fi
        (( i = $i + 1 ))
      done
    done
  done
done

sleep 60