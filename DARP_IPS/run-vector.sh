#!/bin/bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --time=04:30:00
#SBATCH --array=1-2
#SBATCH --output=/dev/null


module load eigen
module load gcc

i=1
for vehicles in manhattan-vehicles
do
  for mode in 1
  do
    for algorithm in 2
    do
      for directory in Instances-120_Epoch
      do
        for instance in 20151211_07-120m_16 20151211_07-120m_17 20151211_07-120m_18
#        for instance in 20150706_07-120m 20150715_07-120m 20150804_07-120m 20150828_07-120m 20150917_07-120m 20150926_07-120m 20151008_07-120m 20151025_07-120m 20151110_07-120m 20151130_07-120m 20151211_07-120m 20151230_07-120m 20160109_07-120m 20160129_07-120m 20160222_07-120m 20160225_07-120m 20160316_07-120m 20160329_07-120m 20160401_07-120m 20160427_07-120m 20160512_07-120m 20160521_07-120m 20160613_07-120m 20160628_07-120m
        do
          for num_vehicles in 2000
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