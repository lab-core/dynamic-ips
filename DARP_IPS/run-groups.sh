#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=36G
#SBATCH --time=4:10:00
#SBATCH --array=1-54
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
G1_vehicle_counts=(1500 1600 1700)
G1_algorithms=(2)
G1_modes=(1)
G1_scenarios=("Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G1_inst_folder="Instances_4h-11"
G1_instances=("20160329_11-240m" "20160628_11-240m")

# G2
G2_vehicle_folder="vehicles_byDemand_w11"
G2_paramfile="AnyParameters"
G2_vehicle_counts=(1600 1700 1800)
G2_algorithms=(2)
G2_modes=(1)
G2_scenarios=("Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G2_inst_folder="Instances_4h-11"
G2_instances=("20150917_11-240m")

# G3
G3_vehicle_folder="vehicles_byDemand_w11"
G3_paramfile="AnyParameters"
G3_vehicle_counts=(1600 1700 1800)
G3_algorithms=(2)
G3_modes=(1)
G3_scenarios=("Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G3_inst_folder="Instances_4h-11"
G3_instances=("20160521_11-240m" "20150926_11-240m")

# G4
G4_vehicle_folder="vehicles_byDemand_w11"
G4_paramfile="AnyParameters"
G4_vehicle_counts=(1000 1100 1200)
G4_algorithms=(2)
G4_modes=(1)
G4_scenarios=("Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G4_inst_folder="Instances_4h-11"
G4_instances=("20160628_11-240m")

# G5
G5_vehicle_folder="vehicles_byDemand_w11"
G5_paramfile="AnyParameters"
G5_vehicle_counts=(1000)
G5_algorithms=(2)
G5_modes=(1)
G5_scenarios=("Rebalance_1")
G5_inst_folder="Instances_4h-11"
G5_instances=("20160628_11-240m")

# Register all for SELECTED_GROUPS=ALL
ALL_GROUPS=(G1 G2)

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