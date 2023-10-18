#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=80G
#SBATCH --time=01:15:00
#SBATCH --array=1-24
#SBATCH --output=/dev/null


module load eigen
module load gcc

i=1
for algorithm in 4
do
  for directory in Instances-300
  do
    for instance in 20151211_17-300m 20151230_17-300m 20160129_17-300m
#  for instance in 20150706_17-300m 20150715_17-300m 20150804_17-300m 20150828_17-300m 20150917_17-300m 20150926_17-300m 20151008_17-300m 20151025_17-300m 20151110_17-300m 20151130_17-300m 20151211_17-300m  20151230_17-300m 20160109_17-300m 20160129_17-300m 20160222_17-300m 20160225_17-300m 20160316_17-300m 20160329_17-300m 20160401_17-300m 20160427_17-300m 20160512_17-300m 20160521_17-300m 20160613_17-300m 20160628_17-300m
    do
      for num_vehicles in 1800
#      for num_vehicles in 1700 1800 1900 2000 2100 2200 2300 2400 2500 2600 2700 2800 2900 3000 3100 3200 3300 3400 3500
      do
        if [ $SLURM_ARRAY_TASK_ID -eq $i ]
        then
          bin/realtime_DARP $directory $instance $num_vehicles $algorithm
#          bin/realtime_DARP $directory $instance $num_vehicles $algorithm > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}.txt" 2>&1
        fi
        (( i = $i + 1 ))
      done
    done
  done
done

sleep 60