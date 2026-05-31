#!/usr/bin/env bash
set -euo pipefail

COMMAND_FILE="${1:?Usage: scripts/run_commands.sh commands/file.txt}"
: "${DRY_RUN:=0}"
: "${START:=1}"
: "${STOP:=}"

if [[ ! -f "$COMMAND_FILE" ]]; then
  echo "[ERROR] Command file not found: $COMMAND_FILE" >&2
  exit 1
fi

total=$(grep -cvE '^\s*(#|$)' "$COMMAND_FILE" || true)
if [[ -z "$STOP" ]]; then
  STOP="$total"
fi

echo "[INFO] Command file: $COMMAND_FILE"
echo "[INFO] Running commands $START..$STOP of $total"

idx=0
while IFS= read -r cmd || [[ -n "$cmd" ]]; do
  [[ -z "${cmd// }" || "$cmd" =~ ^[[:space:]]*# ]] && continue
  idx=$((idx + 1))
  if (( idx < START || idx > STOP )); then
    continue
  fi
  echo "[INFO] $(date) :: command $idx/$total"
  echo "[INFO] $cmd"
  if (( DRY_RUN == 0 )); then
    bash -lc "$cmd"
  else
    echo "[DRY_RUN] would run command $idx"
  fi
done < "$COMMAND_FILE"
