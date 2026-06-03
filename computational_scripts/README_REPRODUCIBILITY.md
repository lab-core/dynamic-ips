# DARP A-CG / B-CG experiments

This folder contains a clean reproducibility layer for the C++ executable. The
Python script does **not** implement the algorithm; it only expands experiment
matrices into command lines that call the C++ binary, usually `bin/realtime_DARP`.

## Directory layout

```text
experiments/
  smoke_test.json
  B_CG.json
  A_CG.json
scripts/
  make_commands.py
  run_one.sh
  run_commands.sh
  run_slurm_array.sh
commands/
results/
```

## Build the C++ code

Build the project so that the executable exists at:

```bash
bin/realtime_DARP
```

## Quick smoke test

Generate one command:

```bash
python scripts/make_commands.py \
  --config experiments/smoke_test.json \
  --commands-out commands/smoke_test.txt \
  --output-dir ../../Outputs/smoke_test
```

Inspect the command without running it:

```bash
cat commands/smoke_test.txt
```

Run the first command locally:

```bash
bash -lc "$(sed -n '1p' commands/smoke_test.txt)"
```

Or run the command file through the helper script:

```bash
scripts/run_commands.sh commands/smoke_test.txt
```

## Generate commands for the B-CG experiments

```bash
python scripts/make_commands.py \
  --config experiments/B_CG.json \
  --commands-out commands/batch_tests.txt \
  --output-dir ../../Outputs/Phase_2
```

Run one subset only, for example the main batch run:

```bash
python scripts/make_commands.py \
  --config experiments/B_CG.json \
  --run commit_notify \
  --commands-out commands/commit_notify.txt \
  --output-dir ../../Outputs/Phase_2
```

Run generated commands locally:

```bash
scripts/run_commands.sh commands/batch_tests.txt
```

For a dry run:

```bash
DRY_RUN=1 scripts/run_commands.sh commands/batch_tests.txt
```

For a small range of commands, for example commands 1 to 5:

```bash
START=1 STOP=5 scripts/run_commands.sh commands/batch_tests.txt
```

## Generate commands for the  A-CG experiments

```bash
python scripts/make_commands.py \
  --config experiments/A_CG.json \
  --commands-out commands/anytime_tests.txt \
  --output-dir ../../Outputs/Phase_3
```

Run generated commands locally:

```bash
scripts/run_commands.sh commands/anytime_tests.txt
```

## Running on an HPC cluster (SLURM)

Generate the commands file with the scratch path as `--output-dir`, then submit from `computational_scripts/`:

```bash
python scripts/make_commands.py \
  --config experiments/B_CG.json \
  --commands-out commands/batch_tests.txt \
  --output-dir /scratch/$USER/dynamic-ips/Phase_2

sbatch --array=1-$(wc -l < commands/batch_tests.txt) \
  scripts/run_slurm_array.sh commands/batch_tests.txt
```

Replace `/scratch/$USER/dynamic-ips` with the actual scratch path on your cluster (e.g. `$SCRATCH`, `$SLURM_TMPDIR`, or a path like `/home/$USER/scratch/dynamic-ips`). The script reads line `SLURM_ARRAY_TASK_ID` from the command file, so each array task runs one experiment. Submit from `computational_scripts/` so that all relative paths (`../bin/realtime_DARP`, `../Riley_Benchmark`, etc.) resolve correctly.

Optional environment variables:

| Variable | Default | Effect |
|---|---|---|
| `DRY_RUN` | `0` | Set to `1` to print commands without executing |
| `LOAD_MODULES` | `1` | Set to `0` to skip the `module load` block |

Dry-run example:

```bash
DRY_RUN=1 sbatch --array=1-5 scripts/run_slurm_array.sh commands/batch_tests.txt
```

## Running a single experiment manually

You can call the executable directly, or use `scripts/run_one.sh`:

```bash
DATA_DIR=../Riley_Benchmark \
VEHICLE_FOLDER=vehicles_warmStart_11 \
INST_FOLDER=Instances_2h-11 \
INSTANCE_NAME=20150926_11-120m \
NUM_VEHICLES=1200 \
VEHICLE_CAPACITY=4 \
MAIN_ALGO=2 \
SOL_MODE=1 \
PARAMFILE=parameters/BatchParameters \
SCENARIO=multiObj_5 \
INITIAL_STATE=1 \
OUTPUT_DIR=../../Outputs/single_run \
scripts/run_one.sh
```

## Filtering commands

`make_commands.py` supports simple filters:

```bash
# one run id
python scripts/make_commands.py --config experiments/A_CG.json --run quality_profile

# selected groups
python scripts/make_commands.py --config experiments/A_CG.json --group aG1,aG2

# selected scenarios
python scripts/make_commands.py --config experiments/B_CG.json --scenario multiObj_5
```

## Auto-discovered instance groups

Some groups intentionally leave `instances` empty and use:

```json
"auto_discover": true
```

For these groups, the script lists all subdirectories under:

```text
<data_dir>/<inst_folder>/
```

For example, `../Riley_Benchmark/Instances_2h-7/`.