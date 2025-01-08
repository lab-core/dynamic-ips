#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --time=2:40:00
#SBATCH --array=1-18
#SBATCH --output=/dev/null

module load eigen
module load gcc

i=1
for vehicles in sufficient_manhattan-vehicles-300
#for vehicles in manhattan-vehicles
do
  for mode in 2
  do
    for algorithm in 6 3
    do
      for directory in Instances_12-14
      do
        # First group of instances with num_vehicles=1000
        for instance in 20150828_12-120m 20151130_12-120m 20160222_12-120m 20151230_12-120m
        do
          num_vehicles=1000
          if [ $SLURM_ARRAY_TASK_ID -eq $i ]
          then
            bin/realtime_DARP $vehicles $directory $instance $num_vehicles $algorithm $mode
#           bin/realtime_DARP $directory $instance $num_vehicles $algorithm $mode > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
          fi
          (( i = $i + 1 ))
        done

        # Second group of instances with num_vehicles=1100
        for instance in 20160316_12-120m 20160512_12-120m
        do
          num_vehicles=1100
          if [ $SLURM_ARRAY_TASK_ID -eq $i ]
          then
            bin/realtime_DARP $vehicles $directory $instance $num_vehicles $algorithm $mode
#           bin/realtime_DARP $directory $instance $num_vehicles $algorithm $mode > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
          fi
          (( i = $i + 1 ))
        done

        # Third group of instances with num_vehicles=1400
        for instance in 20160521_12-120m 20151025_12-120m 20150926_12-120m
        do
          num_vehicles=1400
          if [ $SLURM_ARRAY_TASK_ID -eq $i ]
          then
            bin/realtime_DARP $vehicles $directory $instance $num_vehicles $algorithm $mode
#           bin/realtime_DARP $directory $instance $num_vehicles $algorithm $mode > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
          fi
          (( i = $i + 1 ))
        done

      done
    done
  done
done

sleep 60
