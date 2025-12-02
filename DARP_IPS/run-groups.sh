#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=24G
#SBATCH --time=2:20:00
#SBATCH --array=1-24
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
G1_paramfile="BatchParameters"
G1_vehicle_counts=(1300 1400 1500)
G1_algorithms=(2)
G1_modes=(1)
#G1_scenarios=("dynamic_2_1" "dynamic_3_1" "dynamic_4_1" "commit_1" "no_commit_1" "Ab_drop_1" "Ab_dynamic_1" "Ab_truncate_1")
G1_scenarios=("multiObj_0" "Cust_W3")
G1_inst_folder="Instances_2h-11"
G1_instances=("20150926_11-120m" "20151025_11-120m")

G3_vehicle_folder="vehicles_byDemand_w11"
G3_paramfile="BatchParameters"
G3_vehicle_counts=(900 1000 1100)
G3_algorithms=(2)
G3_modes=(1)
G3_scenarios=("multiObj_0" "Cust_W3")
G3_inst_folder="Instances_2h-11"
G3_instances=("20151230_11-120m")

G4_vehicle_folder="vehicles_byDemand_w11"
G4_paramfile="BatchParameters"
G4_vehicle_counts=(1400 1500 1600)
G4_algorithms=(2)
G4_modes=(1)
G4_scenarios=("multiObj_0" "Cust_W3")
G4_inst_folder="Instances_2h-11"
G4_instances=("20160109_11-120m")

G11_vehicle_folder="vehicles_byDemand_w11"
G11_paramfile="BatchParameters"
G11_vehicle_counts=(1200)
G11_algorithms=(2)
G11_modes=(1)
G11_scenarios=("multiObj_0" "multiObj_1" "multiObj_5" "Cust_W3")
G11_inst_folder="Instances_2h-11"
G11_instances=("20150926_11-120m" "20151025_11-120m")

G31_vehicle_folder="vehicles_byDemand_w11"
G31_paramfile="BatchParameters"
G31_vehicle_counts=(800)
G31_algorithms=(2)
G31_modes=(1)
G31_scenarios=("multiObj_0" "multiObj_1" "multiObj_5" "Cust_W3")
G31_inst_folder="Instances_2h-11"
G31_instances=("20151230_11-120m")

G41_vehicle_folder="vehicles_byDemand_w11"
G41_paramfile="BatchParameters"
G41_vehicle_counts=(1300)
G41_algorithms=(2)
G41_modes=(1)
G41_scenarios=("multiObj_0" "multiObj_1" "multiObj_5" "Cust_W3")
G41_inst_folder="Instances_2h-11"
G41_instances=("20160109_11-120m")

G5_vehicle_folder="vehicles_uniform"
G5_paramfile="BatchParameters"
G5_vehicle_counts=(2000)
G5_algorithms=(2)
G5_modes=(1)
G5_scenarios=("multiObj_0" "Cust_W3")
G5_inst_folder="Instances_2h-7"
G5_instances=("20160401_07-120m" "20160329_07-120m")

# ================================================================
# G2: Automatic group
# ================================================================
#G2_vehicle_folder="vehicles_byDemand"
G2_vehicle_folder="vehicles_uniform"
G2_paramfile="BatchParameters"
G2_vehicle_counts=(2000)
G2_algorithms=(2)
G2_modes=(1)
#G2_scenarios=("initial_0" "initial_1" "pruning_0" "pruning_1" "truncate_0" "truncate_1")
G2_scenarios=("multiObj_0")
#G2_inst_folder="Instances_30s_11"
G2_inst_folder="Instances_2h-7"

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
            jobs+=("$exe --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state 0")
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