#!/usr/bin/env bash
# ==============================================================================
# reproduce-anytime.sh — Reproduce the anytime solver experiments
# ==============================================================================
#
# Runs the DARP anytime solver on the NYC-DARP-Benchmark dataset
# (Zenodo: <insert DOI once published>) and on the Riley_Benchmark dataset
# (Zenodo: https://doi.org/10.5281/zenodo.18745880) for cross-dataset comparisons.
#
# ── Local sequential run ──────────────────────────────────────────────────────
#   bash reproduce-anytime.sh
#   DRY_RUN=1 bash reproduce-anytime.sh              # preview, no execution
#   SELECTED_GROUPS="G_test" bash reproduce-anytime.sh  # single group
#
# ── SLURM cluster ────────────────────────────────────────────────────────────
#   1. Find job count:  DRY_RUN=1 bash reproduce-anytime.sh | grep "Built"
#   2. Set --array=1-N in the SBATCH block below and uncomment those lines
#   3. Submit:          sbatch reproduce-anytime.sh
#
# ── Prerequisites ────────────────────────────────────────────────────────────
#   1. Build the binary — see README.md:
#        mkdir build && cd build && cmake .. && make -j$(nproc)
#   2. Place NYC-DARP-Benchmark at ./NYC-DARP-Benchmark/
#   3. Place Riley_Benchmark at ./Riley_Benchmark/ (for G2h7 group)
#   4. AnyParameters.json must exist in the working directory
# ==============================================================================

## ── SLURM directives: uncomment and adapt to your cluster ───────────────────
##SBATCH --cpus-per-task=16
##SBATCH --mem=46G
##SBATCH --time=4:10:00
##SBATCH --array=1-N          # replace N with job count from the dry run
##SBATCH --output=slurm-%A_%a.out
##SBATCH --error=slurm-%A_%a.err

set -euo pipefail

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }

cd "${SLURM_SUBMIT_DIR:-$PWD}"
: "${DRY_RUN:=0}"
: "${SELECTED_GROUPS:=ALL}"

# ── Output directory ──────────────────────────────────────────────────────────
# Results are written next to each instance by default.
# To redirect outputs to a scratch or HPC directory, set DARP_OUTPUT_DIR:
#   export DARP_OUTPUT_DIR=/scratch/myuser/dynamic-ips
#   bash reproduce-anytime.sh
: "${DARP_OUTPUT_DIR:=}"
out_dir_arg=""
[[ -n "${DARP_OUTPUT_DIR}" ]] && out_dir_arg="--output-dir ${DARP_OUTPUT_DIR}"

# ── Binary ────────────────────────────────────────────────────────────────────
exe="bin/realtime_DARP"
[[ -x "$exe" ]] || {
  echo "[ERROR] Binary not found: $exe" >&2
  echo "[ERROR] Build the project first — see README.md" >&2
  exit 1
}

# ── Shared settings ───────────────────────────────────────────────────────────
readonly PARAMFILE="AnyParameters"
readonly BATCH_ALGOS=(2)
readonly BATCH_MODES=(1)
readonly ANY_ALGOS=(6)
readonly ANY_MODES=(2)

# Scenario sets — names must match entries in AnyParameters.json
readonly SCENS_Rebalance=("Rebalance_no" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5" "Rebalance_6")
readonly SCENS_Profile=("profile_5" "profile_10" "profile_20" "profile_30")
readonly SCENS_reOptimize=("Full_warm" "Basis_warm" "Pool_warm" "Full_cold")
readonly SCENS_reOptimize_keep=("Full_warm_keep" "Basis_warm_keep" "Pool_warm_keep")
readonly SCENS_no_rebalance=("no_rebalance_Basis" "no_rebalance_Pool")
readonly SCENS_process=("Basis_warm_keep" "Greedy")
readonly SCENS_BATCH=("batch")
readonly SCENS_Compare=("Compare_rebalance" "Compare_no_rebalance")
readonly SCENS_Shuttle=("Shuttle_basis" "Shuttle_Greedy")

# Default scenario set for the main paper experiments
readonly SCENS_MAIN=("${SCENS_Rebalance[@]}")

# ── Helper: auto-discover instance directories from the filesystem ────────────
discover_instances() {
  local group="$1"
  local -n _data_dir="${group}_data_dir"
  local -n _inst_folder="${group}_inst_folder"
  local -n _insts="${group}_instances"

  local dir="${_data_dir}/${_inst_folder}"
  info "discover_instances($group): scanning $dir"
  if [[ -d "$dir" ]]; then
    mapfile -t _insts < <(find "$dir" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
    [[ ${#_insts[@]} -gt 0 ]] || warn "$group: no instance directories found in $dir"
  else
    warn "$group: $dir does not exist — group will produce no jobs"
    _insts=()
  fi
}

# ── Group definitions — NYC-DARP-Benchmark (anytime solver) ──────────────────

G1_data_dir="NYC-DARP-Benchmark"; G1_vehicle_folder="vehicles_warmStart_11"
G1_vehicle_counts=(1400 1500 1600 1700); G1_capacity=4
G1_scenarios=("${SCENS_MAIN[@]}"); G1_inst_folder="Instances_4h-11"
G1_instances=("20160512_11-240m"); G1_initial_state=1

G2_data_dir="NYC-DARP-Benchmark"; G2_vehicle_folder="vehicles_warmStart_11"
G2_vehicle_counts=(1450 1550 1650 1750); G2_capacity=4
G2_scenarios=("${SCENS_MAIN[@]}"); G2_inst_folder="Instances_4h-11"
G2_instances=("20150917_11-240m"); G2_initial_state=1

G3_data_dir="NYC-DARP-Benchmark"; G3_vehicle_folder="vehicles_warmStart_11"
G3_vehicle_counts=(1300 1400 1500 1600); G3_capacity=4
G3_scenarios=("${SCENS_MAIN[@]}"); G3_inst_folder="Instances_4h-11"
G3_instances=("20151110_11-240m" "20160628_11-240m"); G3_initial_state=1

# Shuttle service experiments (capacity 7)
G6_data_dir="NYC-DARP-Benchmark"; G6_vehicle_folder="vehicles_warmStart_11"
G6_vehicle_counts=(1200); G6_capacity=7
G6_scenarios=("${SCENS_Shuttle[@]}"); G6_inst_folder="Instances_4h-11"
G6_instances=("20150917_11-240m"); G6_initial_state=1

G7_data_dir="NYC-DARP-Benchmark"; G7_vehicle_folder="vehicles_warmStart_11"
G7_vehicle_counts=(1100); G7_capacity=7
G7_scenarios=("${SCENS_Shuttle[@]}"); G7_inst_folder="Instances_4h-11"
G7_instances=("20151110_11-240m" "20160628_11-240m"); G7_initial_state=1

G8_data_dir="NYC-DARP-Benchmark"; G8_vehicle_folder="vehicles_warmStart_11"
G8_vehicle_counts=(1150); G8_capacity=7
G8_scenarios=("${SCENS_Shuttle[@]}"); G8_inst_folder="Instances_4h-11"
G8_instances=("20160512_11-240m"); G8_initial_state=1

# ── Group definitions — Riley_Benchmark (cross-dataset comparison) ────────────

G2h7_data_dir="Riley_Benchmark";  G2h7_vehicle_folder="vehicles_uniform"
G2h7_vehicle_counts=(2000);        G2h7_capacity=4
G2h7_scenarios=("${SCENS_MAIN[@]}"); G2h7_inst_folder="Instances_2h-7"
G2h7_initial_state=0;              declare -a G2h7_instances
discover_instances G2h7

# Smoke-test group: one instance, fewer vehicles — verify setup before a full run
G_test_data_dir="NYC-DARP-Benchmark"; G_test_vehicle_folder="vehicles_warmStart_11"
G_test_vehicle_counts=(1300 1400);    G_test_capacity=4
G_test_scenarios=("Rebalance_5");     G_test_inst_folder="Instances_4h-11"
G_test_instances=("20160512_11-240m"); G_test_initial_state=1

# ── Build job list ────────────────────────────────────────────────────────────
jobs=()

add_group() {
  local G="$1"
  eval "local data_dir=\${${G}_data_dir}"
  eval "local vehicle_folder=\${${G}_vehicle_folder}"
  eval "local capacity=\${${G}_capacity}"
  eval "local inst_folder=\${${G}_inst_folder}"
  eval "local initial_state=\${${G}_initial_state}"
  eval "local paramfile=\${${G}_paramfile:-$PARAMFILE}"
  eval "local algos=(\"\${${G}_algorithms[@]:-${ANY_ALGOS[*]}}\")"
  eval "local modes=(\"\${${G}_modes[@]:-${ANY_MODES[*]}}\")"
  eval "local counts=(\"\${${G}_vehicle_counts[@]}\")"
  eval "local scens=(\"\${${G}_scenarios[@]}\")"
  eval "local insts=(\"\${${G}_instances[@]:-}\")"

  if [[ ${#insts[@]} -eq 0 || -z "${insts[0]:-}" ]]; then
    warn "Group $G: no instances — skipping"; return
  fi
  if [[ ${#scens[@]} -eq 0 || -z "${scens[0]:-}" ]]; then
    warn "Group $G: no scenarios — skipping"; return
  fi

  local m a s c inst
  for m in "${modes[@]}"; do
    for a in "${algos[@]}"; do
      for s in "${scens[@]}"; do
        for c in "${counts[@]}"; do
          for inst in "${insts[@]}"; do
            jobs+=("$exe --data-dir $data_dir --vehicle-folder $vehicle_folder \
--inst-folder $inst_folder --instance-name $inst --num-vehicles $c \
--vehicle-capacity $capacity --main-algo $a --sol-mode $m \
--paramfile $paramfile --scenario $s $out_dir_arg --initial-state $initial_state")
          done
        done
      done
    done
  done
}

ALL_GROUPS=(G1 G2 G3)

if [[ "$SELECTED_GROUPS" == "ALL" ]]; then
  selected=("${ALL_GROUPS[@]}")
else
  IFS=', ' read -r -a selected <<< "$SELECTED_GROUPS"
fi

for g in "${selected[@]}"; do add_group "$g"; done

total_jobs=${#jobs[@]}
info "PWD=$(pwd)"
info "Built $total_jobs jobs from groups: ${selected[*]}"

# ── Execute ───────────────────────────────────────────────────────────────────
if [[ -n "${SLURM_ARRAY_TASK_ID:-}" ]]; then
  # SLURM array mode: run the single task assigned to this array index
  task_id="$SLURM_ARRAY_TASK_ID"
  if (( task_id < 1 || task_id > total_jobs )); then
    info "Task $task_id is out of range (1..$total_jobs). Nothing to do."
    exit 0
  fi
  cmd="${jobs[$((task_id-1))]}"
  info "$(date) :: Task $task_id/$total_jobs on ${SLURMD_NODENAME:-unknown}"
  info "Running: $cmd"
  (( DRY_RUN )) && { info "[DRY_RUN] skipping execution"; exit 0; }
  srun -- bash -lc "$cmd"
else
  # Local mode: run all jobs sequentially
  info "No SLURM environment — running $total_jobs jobs sequentially"
  for i in "${!jobs[@]}"; do
    cmd="${jobs[$i]}"
    info "Job $((i+1))/$total_jobs: $cmd"
    (( DRY_RUN )) && continue
    eval "$cmd"
  done
fi
