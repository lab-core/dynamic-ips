#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=16G
#SBATCH --time=0:10:00
#SBATCH --array=1-720
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
G1_inst_folder="Instances_2h-11"
G1_instances=("20160329_11-120m")

# ================================================================
# G2: Automatic group
# ================================================================
G2_vehicle_folder="vehicles_byDemand"
G2_paramfile="BatchParameters"
G2_vehicle_counts=(1400)
G2_algorithms=(2)
G2_modes=(1)
#G2_scenarios=("initial_0" "initial_1" "pruning_0" "pruning_1" "truncate_0" "truncate_1")
G2_scenarios=("dropPick_0" "dropPick_1" "dynamic_00" "dynamic_01" "dynamic_10" "dynamic_11")
G2_inst_folder="Instances_30s_11"

# Dynamically discover instances for G2
G2_main_dir="datasets/${G2_inst_folder}"
if [[ -d "$G2_main_dir" ]]; then
  mapfile -t G2_instances < <(find "$G2_main_dir" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
  if [[ ${#G2_instances[@]} -eq 0 ]]; then
    echo "[WARNING] No instances found in $G2_main_dir. G2 will have no jobs."
    G2_instances=()
  else
    echo "[INFO] Found ${#G2_instances[@]} instances in $G2_main_dir: ${G2_instances[*]}"
  fi
else
  echo "[WARNING] Directory $G2_main_dir does not exist. G2 will have no jobs."
  G2_instances=()
fi


# Register all for SELECTED_GROUPS=ALL
ALL_GROUPS=(G2)

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
            jobs+=("$exe --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state 2")
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