import constants as c
import Visualization.visualize_any_CG as vf
from ProcessResults.merge_records import merge_with_averaged_epoch, merge_with_all_iterations, merge_basic, \
    merge_complete
from Visualization.plot_config import PlotConfig
from typing import Iterable, Union


def setup_paths(phase: str, selected_folder: str) -> tuple[str, str]:
    """Setup folder and result paths."""
    folder_name = c.FOLDERS[phase][selected_folder]
    result_folder = f"{phase}/{folder_name}"
    result_path = c.OUTPUT_DIR + result_folder + "/" + "results_epoch_avg.csv"
    return result_folder, result_path



""" rebalance evaluation"""
def plot_rebalance(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    PARAMS       = ['Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2']
    PARAM_LABELS = ['No rebalance', 'Wait ≥ 6min', 'Wait ≥ 5min', 'Wait ≥ 4min', 'Wait ≥ 3min', 'Wait ≥ 2min']
    vf.create_rebalance_histo_by_supply(result_path, config, PARAMS, PARAM_LABELS)
    vf.create_rebalance_vehicle_boxplot_double(result_path, config, PARAMS, PARAM_LABELS)
    vf.create_rebalance_request_boxplot_double(result_path, config, PARAMS, PARAM_LABELS)

""" plot profile"""
def plot_profile(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_profile_boxplot(result_path, config)
    vf.create_anytime_boxplot_double(result_path, config)
    vf.create_anytime_improve_boxplot(result_path, config)

""" reOptimize evaluation"""
def plot_reOptimize(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    NO_POOL    = ['Full_cold', 'Full_warm',      'Pool_warm',      'Basis_warm'     ]
    KEEP_POOL  = ['Full_cold', 'Full_warm_keep', 'Pool_warm_keep', 'Basis_warm_keep']
    SE_METHODS = [
        'Full network (dual cold-start)',
        'Full network (dual warm-start)',
        'Reduction by pool (dual warm-start)',
        'Reduction by basis (dual warm-start)',
    ]
    PARAM_POOL  = ['Pool_warm',  'Pool_warm_keep' ]
    PARAM_BASIS = ['Basis_warm', 'Basis_warm_keep']
    DUAL_LABELS = ['Discard Basis & pool ', 'Keep Basis & Column pool']
    vf.create_reOptimize_boxplot_triple(result_path, config, NO_POOL, KEEP_POOL, SE_METHODS)
    vf.create_reOptimize_dual_profile(result_path, config, PARAM_POOL, PARAM_BASIS, DUAL_LABELS)
    vf.create_reOptimize_time_request_profile(result_path, config, NO_POOL, KEEP_POOL, SE_METHODS)

""" prcocess evaluation"""
def plot_prcocess(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_process_combined_boxplot(result_path, config)


""" rebalance evaluation"""
def plot_rebalance_anytime(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    PARAM_POOL  = ['Pool_warm_keep',  'no_rebalance_Pool' ]
    PARAM_BASIS = ['Basis_warm_keep', 'no_rebalance_Basis']
    LABELS      = ['With Rebalancing', 'Without Rebalancing']
    vf.create_rebalance_anytime_time_request_profile(result_path, config, PARAM_POOL, PARAM_BASIS, LABELS)


# --- mapping from folder name to plotting function ---
PLOT_FUNCTIONS = {
    "rebalance": plot_rebalance,
    "profile": plot_profile,
    "prcocess": plot_prcocess,
    "reOptimize": plot_reOptimize,
    "rebalance_anytime": plot_rebalance_anytime
}


def main(
        name: str,
        phase: str = "Phase_3",
        selected_folders: Union[str, Iterable[str]] = "rebalance",
        gather_data: bool = False,
) -> None:
    """
    Run plots for one or more selected folders.

    Parameters
    ----------
    name : str
        Unused (kept for compatibility, e.g. PyCharm run configuration).
    phase : str
        Phase key used in `c.FOLDERS`.
    selected_folders : str or iterable of str
        Folder key(s) such as 'ablation', 'multiObj', 'nbPicks', etc.
        If set to 'all', runs all available plot functions.
    gather_data : bool
        If True, call `merge_with_averaged_epoch` for each selected folder before plotting.
    """
    config = PlotConfig()

    # Normalize selected_folders to a list
    if isinstance(selected_folders, str):
        if selected_folders == "all":
            folders_to_run = list(PLOT_FUNCTIONS.keys())
        else:
            folders_to_run = [selected_folders]
    else:
        folders_to_run = list(selected_folders)

    for folder in folders_to_run:
        plot_fn = PLOT_FUNCTIONS.get(folder)
        if plot_fn is None:
            print(f"[WARN] No plotting function registered for folder '{folder}'. Skipping.")
            continue

        if gather_data:
            result_folder, _ = setup_paths(phase, folder)
            if folder == "profile":
                merge_with_all_iterations(result_folder)
            else:
                merge_with_averaged_epoch(result_folder)


        print(f"[INFO] Running plots for folder '{folder}' in phase '{phase}'")
        plot_fn(phase=phase, selected_folder=folder, config=config)


if __name__ == "__main__":
    # Use scripts/plot_ACG.py for the CLI entry point:
    #   python scripts/plot_ACG.py --folders rebalance_anytime
    #   python scripts/plot_ACG.py --folders all
    main("", selected_folders="profile", gather_data=False)
