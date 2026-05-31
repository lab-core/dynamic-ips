#!/usr/bin/env bash
# Run one DARP experiment directly. This is useful for README examples and smoke tests.
set -euo pipefail

EXE="${EXE:-../bin/realtime_DARP}"
OUTPUT_DIR="${OUTPUT_DIR:-results/single_run}"

required=(DATA_DIR VEHICLE_FOLDER INST_FOLDER INSTANCE_NAME NUM_VEHICLES VEHICLE_CAPACITY MAIN_ALGO SOL_MODE PARAMFILE SCENARIO INITIAL_STATE)
for name in "${required[@]}"; do
  if [[ -z "${!name:-}" ]]; then
    echo "[ERROR] Missing required environment variable: $name" >&2
    exit 1
  fi
done

mkdir -p "$OUTPUT_DIR"

exec "$EXE" \
  --data-dir "$DATA_DIR" \
  --vehicle-folder "$VEHICLE_FOLDER" \
  --inst-folder "$INST_FOLDER" \
  --instance-name "$INSTANCE_NAME" \
  --num-vehicles "$NUM_VEHICLES" \
  --vehicle-capacity "$VEHICLE_CAPACITY" \
  --main-algo "$MAIN_ALGO" \
  --sol-mode "$SOL_MODE" \
  --paramfile "$PARAMFILE" \
  --scenario "$SCENARIO" \
  --output-dir "$OUTPUT_DIR" \
  --initial-state "$INITIAL_STATE"
