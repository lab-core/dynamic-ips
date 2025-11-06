#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=36G
#SBATCH --time=2:10:00
#SBATCH --array=1-48
#SBATCH --error=slurm-%A_%a.err

# Modules and binary
module load gcc eigen boost gurobi
exe="bin/realtime_DARP"

# Choose which groups to run (comma/space separated) or "ALL".
: "${SELECTED_GROUPS:=ALL}"

# -------------------------
# GROUP DEFINITIONS

# G1
G1_vehicle_folder="vehicles_byDemand_w11"
G1_paramfile="AnyParameters"
G1_vehicle_counts=(1600 1700 1800)
G1_algorithms=(6)
G1_modes=(1)
G1_scenarios=("Dual_1" "Dual_9" "Dual_1_R" "Dual_9_R" "Dual_1_P" "Dual_9_P")
#G1_scenarios=("Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
#G1_scenarios=("Rebalance_no")
G1_inst_folder="Instances_2h-11"
G1_instances=("20160329_11-120m")

# G2
G2_vehicle_folder="vehicles_byDemand_w11"
G2_paramfile="AnyParameters"
G2_vehicle_counts=(1600 1700 1800)
G2_algorithms=(6)
G2_modes=(1)
G2_scenarios=("Dual_1" "Dual_9" "Dual_1_R" "Dual_9_R" "Dual_1_P" "Dual_9_P")
#G2_scenarios=("Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
#G2_scenarios=("Rebalance_no")
G2_inst_folder="Instances_2h-11"
G2_instances=("20150917_11-120m")

# G3
G3_vehicle_folder="vehicles_uniform"
G3_paramfile="BatchParameters"
G3_vehicle_counts=(2000)
G3_algorithms=(2)
G3_modes=(1)
G3_scenarios=("commit")
G3_inst_folder="Instances_2h-7"
G3_instances=("20150706_07-120m" "20150715_07-120m" "20150804_07-120m" "20150828_07-120m"
              "20150917_07-120m" "20150926_07-120m" "20151008_07-120m" "20151025_07-120m"
              "20151110_07-120m" "20151130_07-120m" "20151211_07-120m" "20151230_07-120m"
              "20160109_07-120m" "20160129_07-120m" "20160222_07-120m""20160225_07-120m"
              "20160316_07-120m" "20160329_07-120m" "20160401_07-120m" "20160427_07-120m"
              "20160512_07-120m" "20160521_07-120m" "20160613_07-120m" "20160628_07-120m")

G7_vehicle_folder="vehicles_uniform"
G7_paramfile="AnyParameters"
G7_vehicle_counts=(2000)
G7_algorithms=(2)
G7_modes=(1)
G7_scenarios=("newObj")
G7_inst_folder="Instances_2h-7"
G7_instances=("20150706_07-120m" "20150715_07-120m" "20150804_07-120m" "20150828_07-120m"
              "20150917_07-120m" "20150926_07-120m" "20151008_07-120m" "20151025_07-120m"
              "20151110_07-120m" "20151130_07-120m" "20151211_07-120m" "20151230_07-120m"
              "20160109_07-120m" "20160129_07-120m" "20160222_07-120m""20160225_07-120m"
              "20160316_07-120m" "20160329_07-120m" "20160401_07-120m" "20160427_07-120m"
              "20160512_07-120m" "20160521_07-120m" "20160613_07-120m" "20160628_07-120m")

# G4
G4_vehicle_folder="vehicles_byDemand_w11"
G4_paramfile="AnyParameters"
G4_vehicle_counts=(1000 1100 1200)
G4_algorithms=(6)
G4_modes=(2)
G4_scenarios=("Dual_1_0" "Dual_9")
G4_inst_folder="Instances_4h-11"
G4_instances=("20160628_11-240m")

# G5
G5_vehicle_folder="vehicles_byDemand_w11"
G5_paramfile="AnyParameters"
G5_vehicle_counts=(1300 1400 1500)
G5_algorithms=(6)
G5_modes=(2)
G5_scenarios=("Dual_1_0" "Dual_9")
G5_inst_folder="Instances_4h-11"
G5_instances=("20150926_11-240m" "20160521_11-240m")

# G6
G6_vehicle_folder="vehicles_byDemand_w7"
G6_paramfile="AnyParameters"
G6_vehicle_counts=(1900)
G6_algorithms=(6)
G6_modes=(2)
G6_scenarios=("SP_20")
G6_inst_folder="Instances_16h-7"
G6_instances=("20150917_07-960m")



# Register all for SELECTED_GROUPS=ALL
ALL_GROUPS=(G3 G7)

# -------------------------
# Build job list
# -------------------------
jobs=()

add_group() {
  local G="$1"

  # Indirect scalar lookups
  eval "vehicle_folder=\${${G}_vehicle_folder}"
  eval "paramfile=\${${G}_paramfile}"
  eval "inst_folder=\${${G}_inst_folder}"

  # Indirect array copies (portable to bash 3.2)
  eval "counts_ref=(\"\${${G}_vehicle_counts[@]}\")"
  eval "algos_ref=(\"\${${G}_algorithms[@]}\")"
  eval "modes_ref=(\"\${${G}_modes[@]}\")"
  eval "scens_ref=(\"\${${G}_scenarios[@]}\")"
  eval "insts_ref=(\"\${${G}_instances[@]}\")"

  local m a s c inst
  for m in "${modes_ref[@]}"; do
    for a in "${algos_ref[@]}"; do
      for s in "${scens_ref[@]}"; do
        for c in "${counts_ref[@]}"; do
          for inst in "${insts_ref[@]}"; do
            jobs+=("$exe --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state 1")
          done
        done
      done
    done
  done
}

# Which groups to use
if [[ "$SELECTED_GROUPS" == "ALL" ]]; then
  selected=("${ALL_GROUPS[@]}")
else
  IFS=', ' read -r -a selected <<< "$SELECTED_GROUPS"
fi

# Build commands
for g in "${selected[@]}"; do
  add_group "$g"
done

total_jobs=${#jobs[@]}
echo "[INFO] Built $total_jobs jobs from groups: ${selected[*]}"

# -------------------------
# Run the command for this array task
# -------------------------
task_id="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID is required under sbatch}"
if (( task_id < 1 || task_id > total_jobs )); then
  echo "[INFO] SLURM_ARRAY_TASK_ID=$task_id out of range (1..$total_jobs). Nothing to do."
  exit 0
fi

cmd="${jobs[$((task_id-1))]}"
echo "[INFO] $(date) :: JobID=${SLURM_JOB_ID:-NA} Task $task_id/$total_jobs on ${SLURMD_NODENAME:-unknown}"
echo "[INFO] Running: $cmd"

# Execute on the allocation
srun $cmd