#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=40G
#SBATCH --time=4:20:00
#SBATCH --array=1-96
#SBATCH --output=slurm-%A_%a.out
#SBATCH --error=slurm-%A_%a.err

set -euo pipefail

# Always run from the directory where sbatch was called
cd "${SLURM_SUBMIT_DIR:-$PWD}"

# Default: actually execute on the server (set DRY_RUN=1 to only echo)
: "${DRY_RUN:=0}"
: "${SELECTED_GROUPS:=ALL}"

# ---------
# Modules and binary
# ---------
# Make 'module' work in non-interactive shells (safe if not present)
source /etc/profile.d/modules.sh 2>/dev/null || true
source /usr/share/Modules/init/bash 2>/dev/null || true

if command -v module >/dev/null 2>&1; then
  module purge
  module load gcc eigen boost gurobi
  module list
fi

exe="bin/realtime_DARP"
[[ -x "$exe" ]] || { echo "[ERROR] Executable not found or not executable: $exe" >&2; ls -la bin || true; exit 1; }

# -------------------------
# Shared defaults (DRY)
# -------------------------
readonly BATCH_PARAMFILE="AnyParameters"
readonly BATCH_ALGOS=(3)
readonly BATCH_MODES=(2)

readonly SCENS_Rebalance=("Rebalance_no" "Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
readonly SCENS_anytime=("SP_reoptimize1" "SP_reoptimize2")
readonly SCENS_ISUD=("SP_reoptimize2")
readonly SCENS_BATCH=("batch" "batch_1")

# Bundle scenario for group tests
readonly SCENS_GROUP_TEST=("${SCENS_ISUD[@]}")

# -------------------------
# GROUP DEFINITIONS
# -------------------------
G1_vehicle_folder="vehicles_byDemand_w11"
G1_vehicle_counts=(1300 1400 1500 1600)
G1_scenarios=("${SCENS_GROUP_TEST[@]}")
G1_inst_folder="Instances_4h-11"
G1_instances=("20160512_11-240m" "20151211_11-240m")
G1_initial_state=1

G2_vehicle_folder="vehicles_byDemand_w11"
G2_vehicle_counts=(1400 1500 1600 1700)
G2_scenarios=("${SCENS_GROUP_TEST[@]}")
G2_inst_folder="Instances_4h-11"
G2_instances=("20151008_11-240m" "20150917_11-240m")
G2_initial_state=1

G3_vehicle_folder="vehicles_byDemand_w11"
G3_vehicle_counts=(800 900 1000 1100)
G3_scenarios=("${SCENS_GROUP_TEST[@]}")
G3_inst_folder="Instances_4h-11"
G3_instances=("20151230_11-240m")
G3_initial_state=1

# -------------------------
# Automatic group helpers
# -------------------------
G_test_vehicle_folder="vehicles_byDemand_w11"
G_test_vehicle_counts=(1300 1400)
G_test_algorithms=(2)
G_test_modes=(1)
G_test_scenarios=("Relative_5")
G_test_inst_folder="Instances_2h-11"
G_test_instances=("20150926_11-120m")
G_test_initial_state=1

# -------------------------
# Automatic group helpers
# -------------------------
discover_instances() {
  local group="$1"
  local -n inst_folder_ref="${group}_inst_folder"
  local -n insts_ref="${group}_instances"

  local main_dir="datasets/${inst_folder_ref}"
  info "discover_instances($group): looking in $main_dir"
  if [[ -d "$main_dir" ]]; then
    mapfile -t insts_ref < <(find "$main_dir" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
    if [[ ${#insts_ref[@]} -eq 0 ]]; then
      warn "No instances found in $main_dir. $group will have no jobs."
    else
      info "Found ${#insts_ref[@]} instances for $group: ${insts_ref[*]}"
    fi
  else
    warn "Directory $main_dir does not exist. $group will have no jobs."
    insts_ref=()
  fi
}

# Example auto groups (you had these; not in ALL_GROUPS unless you add them)
G30S_vehicle_folder="vehicles_byDemand"
G30S_vehicle_counts=(1400)
G30S_scenarios=("${SCENS_SMALL_2[@]}")
G30S_inst_folder="Instances_30s_11"
G30S_initial_state=2
discover_instances "G30S"

G2h_7_vehicle_folder="vehicles_uniform"
G2h_7_vehicle_counts=(2000)
G2h_7_scenarios=("${SCENS_GROUP_TEST[@]}")
G2h_7_inst_folder="Instances_2h-7"
G2h_7_initial_state=0
discover_instances "G2h_7"

# -------------------------
# Build job list
# -------------------------
jobs=()

add_group() {
  local G="$1"

  eval "vehicle_folder=\${${G}_vehicle_folder}"
  eval "inst_folder=\${${G}_inst_folder}"
  eval "initial_state=\${${G}_initial_state-}"
  [[ -n "${initial_state:-}" ]] || { echo "[ERROR] Missing ${G}_initial_state" >&2; exit 1; }

  eval "paramfile=\${${G}_paramfile-}"
  [[ -n "${paramfile:-}" ]] || paramfile="$BATCH_PARAMFILE"

  eval "algos_ref=(\"\${${G}_algorithms[@]-}\")"
  if [[ ${#algos_ref[@]} -eq 0 || -z "${algos_ref[0]:-}" ]]; then
    algos_ref=("${BATCH_ALGOS[@]}")
  fi

  eval "modes_ref=(\"\${${G}_modes[@]-}\")"
  if [[ ${#modes_ref[@]} -eq 0 || -z "${modes_ref[0]:-}" ]]; then
    modes_ref=("${BATCH_MODES[@]}")
  fi

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
ALL_GROUPS=(G1 G2)

if [[ "$SELECTED_GROUPS" == "ALL" ]]; then
  selected=("${ALL_GROUPS[@]}")
else
  IFS=', ' read -r -a selected <<< "$SELECTED_GROUPS"
fi

for g in "${selected[@]}"; do
  add_group "$g"
done

total_jobs=${#jobs[@]}
echo "[INFO] PWD=$(pwd)"
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

if (( DRY_RUN == 1 )); then
  echo "[DRY_RUN] Would run: $cmd"
  exit 0
fi

# Execute on the allocation
srun -- bash -lc "$cmd"
