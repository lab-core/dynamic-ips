#!/usr/bin/env bash
#SBATCH --account=def-legraina
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=12
#SBATCH --cpus-per-task=16
#SBATCH --time=4:20:00
#SBATCH --output=/scratch/elamib/slurm/slurm-%j.out
#SBATCH --error=/scratch/elamib/slurm/slurm-%j.err

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }

set -euo pipefail

# Always run from the directory where sbatch was called
cd "${SLURM_SUBMIT_DIR:-$PWD}"

# Redirect HOME to SCRATCH to prevent "Permission denied" errors on compute nodes
export HOME=$SCRATCH

# Default: actually execute on the server (set DRY_RUN=1 to only echo)
: "${DRY_RUN:=0}"
: "${SELECTED_GROUPS:=ALL}"

# ---------
# Modules and binary
# ---------
source /etc/profile.d/modules.sh 2>/dev/null || true
source /usr/share/Modules/init/bash 2>/dev/null || true

if command -v module >/dev/null 2>&1; then
  module purge
  module load gcc eigen boost
  module use /opt/software/commercial/modules
  module load gurobi/13.0.0 2>/dev/null || echo "WARNING: gurobi module not loaded"
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
readonly SCENS_anytime=("SP_Re_2_Pool" "SP_Re_2" "Baseline_Pool" "rebalance_SP1" "Penalty" "rebalance_SP2" "Baseline")
readonly SCENS_MEM=("Penalty" "rebalance_SP2" "Baseline")
readonly SCENS_BATCH=("batch")
readonly SCENS_Compare=("SP_Re_1_Pool" "SP_Re_2_Pool")
readonly SCENS_Dynamic=("Iter_Dynamic_1_S2" "Iter_Dynamic_2_S2" "Iter_Dynamic_3_S2" "Iter_Dynamic_4_S2" "Iter_Fix_1_S1" "Iter_Fix_2_S1" "Iter_Fix_3_S1" "Iter_Fix_4_S1" "Iter_Fix_2_S2" "Iter_Fix_3_S2" "Iter_Fix_4_S2")
readonly SCENS_Fix=("Iter_Fix_2_S2" "Iter_Fix_3_S2" "Iter_Fix_4_S2")

readonly SCENS_GROUP_TEST=("${SCENS_Fix[@]}")

# -------------------------
# GROUP DEFINITIONS MYTEST
# -------------------------
G1_data_dir="my_datasets"
G1_vehicle_folder="vehicles_warmStart_11"
G1_vehicle_counts=(1400 1500 1600 1700)
G1_capacity=4
G1_scenarios=("${SCENS_GROUP_TEST[@]}")
G1_inst_folder="Instances_4h-11"
G1_instances=("20160512_11-240m")
G1_initial_state=1

G2_data_dir="my_datasets"
G2_vehicle_folder="vehicles_warmStart_11"
G2_vehicle_counts=(1450 1550 1650 1750)
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
            jobs+=("$exe --data-dir $data_dir --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --vehicle-capacity $capacity --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 2 --initial-state $initial_state")
          done
        done
      done
    done
  done
}

ALL_GROUPS=(G6 G7 G8)

if [[ "$SELECTED_GROUPS" == "ALL" ]]; then
  selected=("${ALL_GROUPS[@]}")
else
  IFS=', ' read -r -a selected <<< "$SELECTED_GROUPS"
fi

for g in "${selected[@]}"; do
  add_group "$g"
done

total_jobs=${#jobs[@]}
echo "[INFO] Built $total_jobs jobs to run concurrently on 1 node"

# -------------------------
# Run all built jobs concurrently (Task Bundling)
# -------------------------
if (( DRY_RUN == 1 )); then
  for cmd in "${jobs[@]}"; do
    echo "[DRY_RUN] Would run: $cmd"
  done
  exit 0
fi

# Launch each job in the background using srun
for cmd in "${jobs[@]}"; do
  echo "[INFO] Launching: $cmd"
  # --exact ensures srun only grabs the 16 cores needed for this specific task
  srun --ntasks=1 --cpus-per-task=16 --exact bash -lc "$cmd" &
done

# Wait for all background tasks to finish before ending the SLURM allocation
wait
echo "[INFO] All tasks completed."