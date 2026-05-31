#!/usr/bin/env bash
#SBATCH --job-name=darp-repro
#SBATCH --cpus-per-task=16
#SBATCH --mem=36G
#SBATCH --time=2:20:00
#SBATCH --output=slurm-%A_%a.out
#SBATCH --error=slurm-%A_%a.err

set -euo pipefail

COMMAND_FILE="${1:?Usage: sbatch --array=1-N scripts/run_slurm_array.sh commands/file.txt}"
DRY_RUN="${DRY_RUN:-0}"
LOAD_MODULES="${LOAD_MODULES:-1}"

cd "${SLURM_SUBMIT_DIR:-$PWD}"

if [[ ! -f "$COMMAND_FILE" ]]; then
  echo "[ERROR] command file not found: $COMMAND_FILE" >&2
  exit 1
fi

if [[ "$LOAD_MODULES" == "1" ]]; then
  source /etc/profile.d/modules.sh 2>/dev/null || true
  source /usr/share/Modules/init/bash 2>/dev/null || true
  if command -v module >/dev/null 2>&1; then
    module purge
    module load gcc eigen boost gurobi
    module list
  fi
fi

: "${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID is required. Submit with --array=1-N.}"
cmd="$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$COMMAND_FILE")"

if [[ -z "${cmd// }" ]]; then
  echo "[INFO] No command at line ${SLURM_ARRAY_TASK_ID}; exiting."
  exit 0
fi

export OMP_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

echo "[INFO] $(date) :: JobID=${SLURM_JOB_ID:-NA} Task=${SLURM_ARRAY_TASK_ID} Node=${SLURMD_NODENAME:-unknown}"
echo "[INFO] PWD=$(pwd)"
echo "[INFO] Command: $cmd"

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[DRY_RUN] Would run: $cmd"
  exit 0
fi

srun -- bash -lc "$cmd"
