#!/usr/bin/env bash
#SBATCH --cpus-per-task=16
#SBATCH --mem=30G
#SBATCH --time=4:10:00
#SBATCH --array=1-54
#SBATCH --output=/dev/null

# Modules and binary
module load gcc eigen
exe="bin/realtime_DARP"

# Choose which groups to run (comma/space separated) or "ALL".
# Override at submit: sbatch --export=SELECTED_GROUPS="G1,G2,G3" run.sh
: "${SELECTED_GROUPS:=ALL}"

# -------------------------
# GROUP DEFINITIONS

# G1 → num_vehicles=1000 with its instances
G1_vehicle_folder="vehicles_byDemand_w11"
G1_paramfile="AnyParameters"
G1_vehicle_counts=(1300 1400 1500)
G1_algorithms=(6)
G1_modes=(2)
G1_scenarios=("Rebalance_no" "Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G1_inst_folder="Instances_4h-11"
G1_instances=("20160521_11-240m" "20150926_11-240m")

# G2 → num_vehicles=1100 with its instances
G2_vehicle_folder="vehicles_byDemand_w11"
G2_paramfile="AnyParameters"
G2_vehicle_counts=(1000 1100 1200)
G2_algorithms=(6)
G2_modes=(2)
G2_scenarios=("Rebalance_no" "Rebalance_1" "Rebalance_2" "Rebalance_3" "Rebalance_4" "Rebalance_5")
G2_inst_folder="Instances_4h-11"
G2_instances=("20160628_11-240m")


# Register all for SELECTED_GROUPS=ALL
ALL_GROUPS=(G1 G2)

# -------------------------
# Build job list
# -------------------------
jobs=()

add_group() {
  local G="$1"
  local vf_var="${G}_vehicle_folder"; local vehicle_folder="${!vf_var}"
  local pf_var="${G}_paramfile";      local paramfile="${!pf_var}"
  local if_var="${G}_inst_folder";    local inst_folder="${!if_var}"

  declare -n counts_ref="${G}_vehicle_counts"
  declare -n algos_ref="${G}_algorithms"
  declare -n modes_ref="${G}_modes"
  declare -n scens_ref="${G}_scenarios"
  declare -n insts_ref="${G}_instances"

  local m a s c inst
  for m in "${modes_ref[@]}"; do
    for a in "${algos_ref[@]}"; do
      for s in "${scens_ref[@]}"; do
        for c in "${counts_ref[@]}"; do
          for inst in "${insts_ref[@]}"; do
            jobs+=("$exe --vehicle-folder $vehicle_folder --inst-folder $inst_folder --instance-name $inst --num-vehicles $c --main-algo $a --sol-mode $m --paramfile $paramfile --scenario $s --save-scratch 1 --initial-state 1")
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
task_id="${SLURM_ARRAY_TASK_ID:-1}"   # default 1 for quick local tests
if (( task_id < 1 || task_id > total_jobs )); then
  echo "[INFO] SLURM_ARRAY_TASK_ID=$task_id out of range (1..$total_jobs)."
  exit 0
fi

cmd="${jobs[$((task_id-1))]}"
echo "[INFO] $(date) :: Task $task_id/$total_jobs"
echo "[INFO] $cmd"
srun $cmd
