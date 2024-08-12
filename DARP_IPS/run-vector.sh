#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=80G
#SBATCH --time=2:30:00
#SBATCH --array=1-24
#SBATCH --output=/dev/null


module load eigen
module load gcc

i=1
#for vehicles in sufficient_manhattan-vehicles-300
for vehicles in manhattan-vehicles
do
  for mode in 1
  do
    for algorithm in 2
    do
      for directory in Instances-120
      do
#        for instance in 20160109_11-300m 20150706_11-300m 20160316_11-300m
#        for instance in 20150828_12-120m 20151130_12-120m 20160222_12-120m
#        for instance in 20160316_12-120m 20151110_12-120m 20160512_12-120m
#        for instance in 20160521_12-120m 20151025_12-120m 20150926_12-120m
        for instance in 20150706_07-120m 20150715_07-120m 20150804_07-120m 20150828_07-120m 20150917_07-120m 20150926_07-120m 20151008_07-120m 20151025_07-120m 20151110_07-120m 20151130_07-120m 20151211_07-120m 20151230_07-120m 20160109_07-120m 20160129_07-120m 20160222_07-120m 20160225_07-120m 20160316_07-120m 20160329_07-120m 20160401_07-120m 20160427_07-120m 20160512_07-120m 20160521_07-120m 20160613_07-120m 20160628_07-120m
        # for instance in 20150706_07-120m 20150828_07-120m 20160613_07-120m 20160628_07-120m
#        for instance in 20150715_07-120m 20150804_07-120m 20150917_07-120m 20151008_07-120m 20151110_07-120m 20151130_07-120m 20151211_07-120m 20160129_07-120m 20160222_07-120m 20160225_07-120m 20160316_07-120m 20160329_07-120m 20160401_07-120m 20160427_07-120m 20160512_07-120m
#        for instance in 20150926_07-120m 20151025_07-120m 20151230_07-120m 20160109_07-120m 20160521_07-120m

        do
          for num_vehicles in 2000
#          for num_vehicles in 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000 2100 2200 2300 2400 2500
          do
            if [ $SLURM_ARRAY_TASK_ID -eq $i ]
            then
              bin/realtime_DARP $vehicles $directory $instance $num_vehicles $algorithm $mode
#             bin/realtime_DARP $directory $instance $num_vehicles $algorithm $mode > "/scratch/amirelah/dynamic-ips/${directory}_${instance}_${num_vehicles}_${algorithm}_${mode}.txt" 2>&1
            fi
            (( i = $i + 1 ))
          done
        done
      done
    done
  done
done

sleep 60