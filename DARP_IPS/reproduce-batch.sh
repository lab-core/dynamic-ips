#!/usr/bin/env bash
# ==============================================================================
# reproduce-batch.sh — Reproduce the batch solver experiments
# ==============================================================================
#
# Runs the DARP column-generation solver in batch mode on the
# Riley_Benchmark dataset (Zenodo: https://doi.org/10.5281/zenodo.18745880).
#
# ── Local sequential run ──────────────────────────────────────────────────────
#   bash reproduce-batch.sh
#   DRY_RUN=1 bash reproduce-batch.sh           # preview commands, no execution
#   SELECTED_GROUPS="G_test" bash reproduce-batch.sh  # single group
#
# ── SLURM cluster ────────────────────────────────────────────────────────────
#   1. Find job count:  DRY_RUN=1 bash reproduce-batch.sh | grep "Built"
#   2. Set --array=1-N in the SBATCH block below and uncomment those lines
#   3. Submit:          sbatch reproduce-batch.sh
#
# ── Prerequisites ────────────────────────────────────────────────────────────
#   1. Build the binary — see README.md:
#        mkdir build && cd build && cmake .. && make -j$(nproc)
#   2. Place the Riley_Benchmark dataset at ./Riley_Benchmark/
#   3. BatchParameters.json must exist in the working directory
# ==============================================================================

## ── SLURM directives: uncomment and adapt to your cluster ───────────────────
##SBATCH --cpus-per-task=16
##SBATCH --mem=36G
##SBATCH --time=2:20:00
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
#   bash reproduce-batch.sh
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
readonly PARAMFILE="BatchParameters"
readonly BATCH_ALGOS=(2)
readonly BATCH_MODES=(1)

# Scenario sets — names must match entries in BatchParameters.json
readonly SCENS_PICKUP=("nbPick1" "nbPick2" "nbPick3" "nbPick4")
readonly SCENS_INIT=("initial_0" "initial_1" "pruning_0" "pruning_1")
readonly SCENS_SMALL=("truncate_0" "truncate_1" "dropPick_0" "dropPick_1" "dynamic_0" "dynamic_1")
readonly SCENS_COMMIT=("no_commit_0" "no_commit_1" "commit_0" "commit_1")
readonly SCENS_ABLATION=("Ab_drop_0" "Ab_drop_1" "Ab_dynamic_0" "Ab_dynamic_1" "Ab_truncate_0" "Ab_truncate_1")
readonly SCENS_MULTI_OBJ=("multiObj_0" "multiObj_1" "multiObj_5" "Cust_W3" "Relative" "Relative_5" "Total" "Jung")

# Default scenario set used for the main paper experiments
readonly SCENS_MAIN=("${SCENS_ABLATION[@]}")

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

# ── Group definitions ─────────────────────────────────────────────────────────
# Each group corresponds to a set of experiments in the paper.
# Adjust vehicle counts and scenarios to reproduce specific tables/figures.

G1_data_dir="Riley_Benchmark";     G1_vehicle_folder="vehicles_byDemand_w11"
G1_vehicle_counts=(1200 1300 1400 1500);  G1_capacity=4
G1_scenarios=("${SCENS_MAIN[@]}"); G1_inst_folder="Instances_2h-11"
G1_instances=("20150926_11-120m" "20151025_11-120m"); G1_initial_state=1

G2_data_dir="Riley_Benchmark";     G2_vehicle_folder="vehicles_byDemand_w11"
G2_vehicle_counts=(1300 1400 1500 1600);  G2_capacity=4
G2_scenarios=("${SCENS_MAIN[@]}"); G2_inst_folder="Instances_2h-11"
G2_instances=("20160109_11-120m"); G2_initial_state=1

G3_data_dir="Riley_Benchmark";     G3_vehicle_folder="vehicles_byDemand_w11"
G3_vehicle_counts=(800 900 1000 1100);   G3_capacity=4
G3_scenarios=("${SCENS_MAIN[@]}"); G3_inst_folder="Instances_2h-11"
G3_instances=("20151230_11-120m"); G3_initial_state=1

# 30-second epoch instances (instances auto-discovered)
G30S_data_dir="Riley_Benchmark";   G30S_vehicle_folder="vehicles_byDemand"
G30S_vehicle_counts=(1400);        G30S_capacity=4
G30S_scenarios=("${SCENS_SMALL[@]}"); G30S_inst_folder="Instances_30s_11"
G30S_initial_state=2;              declare -a G30S_instances
discover_instances G30S

# 2-hour instances with 7 zones (instances auto-discovered)
G2h_7_data_dir="Riley_Benchmark"; G2h_7_vehicle_folder="vehicles_uniform"
G2h_7_vehicle_counts=(2000);      G2h_7_capacity=4
G2h_7_scenarios=("${SCENS_MAIN[@]}"); G2h_7_inst_folder="Instances_2h-7"
G2h_7_initial_state=0;            declare -a G2h_7_instances
discover_instances G2h_7

# Smoke-test group: one instance, fewer vehicles — verify setup before a full run
G_test_data_dir="Riley_Benchmark"; G_test_vehicle_folder="vehicles_byDemand_w11"
G_test_vehicle_counts=(1300);      G_test_capacity=4
G_test_scenarios=("Ab_drop_1");   G_test_inst_folder="Instances_2h-11"
G_test_instances=("20150926_11-120m"); G_test_initial_state=1

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
  eval "local algos=(\"\${${G}_algorithms[@]:-${BATCH_ALGOS[*]}}\")"
  eval "local modes=(\"\${${G}_modes[@]:-${BATCH_MODES[*]}}\")"
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
