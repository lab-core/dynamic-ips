#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=64G
#SBATCH --time=2:20:00
#SBATCH --array=1-24
#SBATCH --error=slurm-%A_%a.err

# Modules and binary
module load gcc eigen boost gurobi
exe="bin/realtime_DARP"

# Choose which groups to run (comma/space separated) or "ALL".
: "${SELECTED_GROUPS:=ALL}"

# -------------------------
# Shared defaults (DRY)
# -------------------------
readonly BATCH_PARAMFILE="BatchParameters."
readonly BATCH_ALGOS=(2)
readonly BATCH_MODES=(1)

readonly SCENS_PICKUP=("nbPick1" "nbPick2" "nbPick3" "nbPick4")
readonly SCENS_SMALL_1=("initial_0" "initial_1" "pruning_0" "pruning_1")
readonly SCENS_SMALL_2=("truncate_0" "truncate_1" "dropPick_0" "dropPick_1" "dynamic_0" "dynamic_1")
readonly SCENS_COMMIT=("no_commit_0" "no_commit_1" "commit_0" "commit_1")
readonly SCENS_ABLATION=("Ab_drop_1" "Ab_drop_0" "Ab_dynamic_1" "Ab_dynamic_0" "Ab_truncate_1" "Ab_truncate_0")
readonly SCENS_MULTI_OBJ=("multiObj_0" "multiObj_1" "multiObj_5")
readonly SCENS_COMPARE=("multiObj_5")
readonly SCENS_W3=("Cust_W3")
readonly SCENS_W5=("Relative" "Relative_5")

# Bundle scenarion for group tests
readonly SCENS_GROUP_TEST=( "${SCENS_W5[@]}")

# -------------------------
# GROUP DEFINITIONS
# -------------------------

# ================================================================
# Group tests for 2h-11
# ================================================================
G1_vehicle_folder="vehicles_byDemand_w11"
G1_vehicle_counts=(1300 1400 1500)
G1_scenarios=("${SCENS_GROUP_TEST[@]}")
G1_inst_folder="Instances_2h-11"
G1_instances=("20150926_11-120m" "20151025_11-120m")
G1_initial_state=1

G2_vehicle_folder="vehicles_byDemand_w11"
G2_vehicle_counts=(1400 1500 1600)
G2_scenarios=("${SCENS_GROUP_TEST[@]}")
G2_inst_folder="Instances_2h-11"
G2_instances=("20160109_11-120m")
G2_initial_state=1

G3_vehicle_folder="vehicles_byDemand_w11"
G3_vehicle_counts=(900 1000 1100)
G3_scenarios=("${SCENS_GROUP_TEST[@]}")
G3_inst_folder="Instances_2h-11"
G3_instances=("20151230_11-120m")
G3_initial_state=1

# ================================================================
# Arbitrary test definitions
# ================================================================

G_test_vehicle_folder="vehicles_byDemand_w11"
G_test_vehicle_counts=(1300)
G_test_algorithms=(2)
G_test_modes=(1)
G_test_scenarios=("Ab_drop_0" "Ab_dynamic_0" "Ab_truncate_0")
G_test_inst_folder="Instances_2h-11"
G_test_instances=("20150926_11-120m" "20151025_11-120m")
G_test_initial_state=1

# ================================================================
# Automatic group
# ================================================================
# Dynamically discover instances for G2
discover_instances() {
  local group="$1"
  local -n inst_folder_ref="${group}_inst_folder"
  local -n insts_ref="${group}_instances"

  local main_dir="datasets/${inst_folder_ref}"
  if [[ -d "$main_dir" ]]; then
    mapfile -t insts_ref < <(find "$main_dir" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
    if [[ ${#insts_ref[@]} -eq 0 ]]; then
      echo "[WARNING] No instances found in $main_dir. $group will have no jobs."
    else
      echo "[INFO] Found ${#insts_ref[@]} instances in $main_dir for $group."
    fi
  else
    echo "[WARNING] Directory $main_dir does not exist. $group will have no jobs."
    insts_ref=()
  fi
}
# G30S: Small size instances
G30S_vehicle_folder="vehicles_byDemand"
G30S_vehicle_counts=(1400)
G30S_scenarios=("${SCENS_SMALL_2[@]}")
G30S_inst_folder="Instances_30s_11"
G30S_initial_state=2
discover_instances "G30S"

# G2h_7: 2h instances (Riley)
G2h_7_vehicle_folder="vehicles_uniform"
G2h_7_vehicle_counts=(2000)
G2h_7_scenarios=("${SCENS_COMPARE[@]}")
G2h_7_inst_folder="Instances_2h-7"
G2h_7_initial_state=0
discover_instances "G2h_7"

# Register all for SELECTED_GROUPS=ALL
ALL_GROUPS=(G1 G2 G3)

# -------------------------
# Build job list
# -------------------------
jobs=()

add_group() {
  local G="$1"

  # required scalars
  eval "vehicle_folder=\${${G}_vehicle_folder}"
  eval "inst_folder=\${${G}_inst_folder}"
  eval "initial_state=\${${G}_initial_state-}"
  [[ -n "${initial_state:-}" ]] || { echo "[ERROR] Missing ${G}_initial_state" >&2; exit 1; }

  # paramfile: group override or default
  eval "paramfile=\${${G}_paramfile-}"
  [[ -n "${paramfile:-}" ]] || paramfile="$BATCH_PARAMFILE"

  # arrays: group override or default
  eval "algos_ref=(\"\${${G}_algorithms[@]-}\")"
  [[ ${#algos_ref[@]} -gt 0 ]] || algos_ref=("${BATCH_ALGOS[@]}")

  eval "modes_ref=(\"\${${G}_modes[@]-}\")"
  [[ ${#modes_ref[@]} -gt 0 ]] || modes_ref=("${BATCH_MODES[@]}")

  # other required arrays
  eval "counts_ref=(\"\${${G}_vehicle_counts[@]}\")"
  eval "scens_ref=(\"\${${G}_scenarios[@]}\")"
  eval "insts_ref=(\"\${${G}_instances[@]}\")"

  local m a s c inst
  for m in "${modes_ref[@]}"; do
    for a in "${algos_ref[@]}"; do
      for s in "${scens_ref[@]}"; do
        for c in "${counts_ref[@]}"; do
          for inst in "${insts_ref[@]}"; do
            jobs+=("$exe --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state $initial_state")
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
srun -- bash -lc "$cmd"