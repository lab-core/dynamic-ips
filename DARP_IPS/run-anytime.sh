#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=46G
#SBATCH --time=4:20:00
#SBATCH --array=1-3
#SBATCH --output=slurm-%A_%a.out
#SBATCH --error=slurm-%A_%a.err

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }

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
readonly BATCH_ALGOS=(6)
readonly BATCH_MODES=(2)

readonly SCENS_Rebalance=("Rebalance_no" "Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5" "Rebalance_6" "Rebalance_7")
readonly SCENS_anytime=("SP_Re_1_Pool" "SP_Re_1" "SP_Re_2_Pool" "SP_Re_2" "Baseline_Pool" "rebalance_SP1" "Penalty" "rebalance_SP2" "Baseline")
readonly SCENS_MEM=("Penalty" "rebalance_SP2" "Baseline")
readonly SCENS_BATCH=("rebalance_SP2")
readonly SCENS_Compare=("SP_Re_1_Pool" "SP_Re_2_Pool")
readonly SCENS_Dynamic=("Iter_Dynamic_1_S2" "Iter_Dynamic_2_S2" "Iter_Dynamic_3_S2" "Iter_Dynamic_4_S2" "Iter_Fix_1_S1" "Iter_Fix_2_S1" "Iter_Fix_3_S1" "Iter_Fix_4_S1" "Iter_Fix_2_S2" "Iter_Fix_3_S2" "Iter_Fix_4_S2")
readonly SCENS_Fix=("Iter_Fix_1_S1" "Iter_Fix_2_S1" "Iter_Fix_3_S1" "Iter_Fix_4_S1" "Iter_Fix_2_S2" "Iter_Fix_3_S2" "Iter_Fix_4_S2")

# Bundle scenario for group tests
readonly SCENS_GROUP_TEST=("${SCENS_BATCH[@]}")

# -------------------------
# GROUP DEFINITIONS MYTEST
# -------------------------
G1_data_dir="my_datasets"
G1_vehicle_folder="vehicles_warmStart_11"
G1_vehicle_counts=(1400 1500)
G1_capacity=4
G1_scenarios=("${SCENS_GROUP_TEST[@]}")
G1_inst_folder="Instances_4h-11"
G1_instances=("20160512_11-240m")
G1_initial_state=1

G2_data_dir="my_datasets"
G2_vehicle_folder="vehicles_warmStart_11"
G2_vehicle_counts=(1450)
G2_capacity=4
G2_scenarios=("${SCENS_GROUP_TEST[@]}")
G2_inst_folder="Instances_4h-11"
G2_instances=("20150917_11-240m")
G2_initial_state=1

G3_data_dir="my_datasets"
G3_vehicle_folder="vehicles_warmStart_11"
G3_vehicle_counts=(1300 1400 1500 1600)
G3_capacity=4
G3_scenarios=("${SCENS_GROUP_TEST[@]}")
G3_inst_folder="Instances_4h-11"
G3_instances=("20151110_11-240m" "20160628_11-240m")
G3_initial_state=1


# -------------------------
# GROUP DEFINITIONS RILEY
# -------------------------
G4_data_dir="datasets"
G4_vehicle_folder="vehicles_warmStart_11"
G4_vehicle_counts=(1200 1300 1400 1500)
G4_capacity=4
G4_scenarios=("${SCENS_GROUP_TEST[@]}")
G4_inst_folder="Instances_4h-11"
G4_instances=("20160521_11-240m" "20150926_11-240m" "20151025_11-240m")
G4_initial_state=1

G5_data_dir="datasets"
G5_vehicle_folder="vehicles_warmStart_11"
G5_vehicle_counts=(1000 1100 1200 1300)
G5_capacity=4
G5_scenarios=("${SCENS_GROUP_TEST[@]}")
G5_inst_folder="Instances_4h-11"
G5_instances=("20151008_11-240m")
G5_initial_state=1

# -------------------------
# GROUP DEFINITIONS Shuttle Service
# -------------------------
G6_data_dir="my_datasets"
G6_vehicle_folder="vehicles_warmStart_11"
G6_vehicle_counts=(1200)
G6_capacity=7
G6_scenarios=("${SCENS_GROUP_TEST[@]}")
G6_inst_folder="Instances_4h-11"
G6_instances=("20150917_11-240m")
G6_initial_state=1

G7_data_dir="my_datasets"
G7_vehicle_folder="vehicles_warmStart_11"
G7_vehicle_counts=(1100)
G7_capacity=7
G7_scenarios=("${SCENS_GROUP_TEST[@]}")
G7_inst_folder="Instances_4h-11"
G7_instances=("20151110_11-240m" "20160628_11-240m")
G7_initial_state=1

G8_data_dir="my_datasets"
G8_vehicle_folder="vehicles_warmStart_11"
G8_vehicle_counts=(1150)
G8_capacity=7
G8_scenarios=("${SCENS_GROUP_TEST[@]}")
G8_inst_folder="Instances_4h-11"
G8_instances=("20160512_11-240m")
G8_initial_state=1

# -------------------------
# Automatic group helpers
# -------------------------
G_test_data_dir="my_datasets"
G_test_vehicle_folder="vehicles_warmStart_11"
G_test_vehicle_counts=(1300 1400)
G_test_capacity=4
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
  local -n data_dir_ref="${group}_data_dir"
  local -n inst_folder_ref="${group}_inst_folder"
  local -n insts_ref="${group}_instances"

  local main_dir="${data_dir_ref}/${inst_folder_ref}"

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
G30S_data_dir="datasets"
G30S_vehicle_folder="vehicles_warmStart_11"
G30S_vehicle_counts=(1400)
G30S_capacity=4
G30S_scenarios=("${SCENS_GROUP_TEST[@]}")
G30S_inst_folder="Instances_30s"
G30S_initial_state=2
discover_instances "G30S"

G2h7_data_dir="datasets"
G2h7_vehicle_folder="vehicles_uniform"
G2h7_vehicle_counts=(2000 1500)
G2h7_capacity=4
G2h7_scenarios=("${SCENS_GROUP_TEST[@]}")
G2h7_inst_folder="Instances_2h-7"
G2h7_initial_state=0
discover_instances "G2h7"

# -------------------------
# Build job list
# -------------------------
jobs=()

add_group() {
  local G="$1"

  eval "data_dir=\${${G}_data_dir}"
  eval "vehicle_folder=\${${G}_vehicle_folder}"
  eval "capacity=\${${G}_capacity}"
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
            jobs+=("$exe --data-dir $data_dir --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --vehicle-capacity $capacity --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state $initial_state")
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
