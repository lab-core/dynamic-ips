import constants as c
import Visualization.visualize_batch_CG as vf
import Visualization.visualize_batch_custom as vc
from ProcessResults.merge_records import merge_with_averaged_epoch, merge_with_all_iterations
from Visualization.plot_config import PlotConfig
from typing import Iterable, Union

from Visualization.visualize_others import create_pruning_labels_and_time_figure


def setup_paths(phase: str, selected_folder: str) -> tuple[str, str]:
    """Setup folder and result paths."""
    folder_name = c.FOLDERS[phase][selected_folder]
    result_folder = f"{phase}/{folder_name}"
    result_path = c.OUTPUT_DIR + result_folder + "/" + "results_epoch_avg.csv"
    return result_folder, result_path

""" Pickup Limit"""
def plot_nbPick(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_nbPick_barplot_double(result_path, config)
    vf.create_nbPick_time_barplot_double(result_path, config)
    vf.create_nbPick_wait_barplot_double(result_path, config)
    vf.create_nbPick_obj_barplot_double(result_path, config)

""" Pruning Strategy"""
def plot_pruning(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    create_pruning_labels_and_time_figure(result_path, config)
    vf.create_pruning_scatter_plot_double(result_path, config)
    vf.create_pruning_time_boxplot_double(result_path, config)

""" Truncated Labeling"""
def plot_truncated(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_truncate_time_boxplot(result_path, config)
    vc.create_truncate_objective_scatter(result_path, config)

""" DropPick """
def plot_dropPick(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_dropPick_wait_time_baxplot_double(result_path, config)
    vf.create_dropPick_time_boxplot_double(result_path, config)
    vf.create_dropPick_wait_barplot_double(result_path, config)

""" Dynamic Pickups"""
def plot_dynamic(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_runtime_iter_box_violin(result_path, config)
    vf.create_dynamic_iter_violinplot_double(result_path, config)
    vf.create_dynamic_wait_time_baxplot_double(result_path, config)
    vf.create_dynamic_time_boxplot_double(result_path, config)
    vf.create_dynamic_avgtime_boxplot_double(result_path, config)
    vc.create_dynamic_iteration_bubble_plot_double(result_path, config)


""" Commit evaluation"""
def plot_commit(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_commit_wait_response_baxplot_double(result_path, config)
    vf.create_commit_wait_response_baxplot_group(result_path, config)
    vf.create_commit_wait_boxplot_double(result_path, config)
    vf.create_commit_response_boxplot_double(result_path, config)

""" Ablation evaluation"""
def plot_ablation(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    PARAM_MAP    = c.param_to_setting          # dict: paramFile value → display label
    PARAM_LABELS = c.param_to_setting_labels   # list of display labels (ordered)
    vf.create_ablation_iter_violinplot_single(result_path, config)
    vf.create_ablation_request_boxplot_fourth(result_path, config, PARAM_MAP, PARAM_LABELS)
    vf.create_ablation_request_boxplot_triple(result_path, config)
    vf.create_ablation_iter_violinplot_double(result_path, config)
    vf.create_ablation_wait_boxplot_double(result_path, config)
    vf.create_ablation_time_boxplot_double(result_path, config)

""" multiObj evaluation"""
def plot_multiObj(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_multiObj_vehicle_std_plot(result_path, config)
    vf.create_multiObj_multi_std_plot(result_path, config)
    vf.create_multiObj_time_request_profile(result_path, config)

    vf.create_multiObj_vehicle_KPI_boxplot_double(result_path, config)
    vf.create_multiObj_request_boxplot_double(result_path, config)

    vf.create_multiObj_request_boxplot_triple(result_path, config)
    vf.create_multiObj_time_request_profile(result_path, config)
    """
    vf.create_multiObj_gap_violinplots(result_path, config)
    
    vf.create_multiObj_request_boxplot_double(result_path, config)
    vf.create_multiObj_request_boxplot_triple(result_path, config)
    vf.create_multiObj_vehicle_KPI_boxplot(result_path, config)
    
    vf.create_multiObj_time_dual_profile(result_path, config)
    vf.create_epoch_vehicle_boxplot(result_path, config)
    
    vf.create_multiObj_time_boxplot_double(result_path, config)
    vf.create_multiObj_wait_grouped(result_path, config)
    vf.create_multiObj_tripdelay_grouped(result_path, config)
    vf.create_multiObj_passenger_grouped(result_path, config)
    vf.create_multiObj_idle_grouped(result_path, config)
    
    vf.create_multiObj_passenger_profile(result_path, config)
    vf.create_multiObj_wait_delay_profile(result_path, config)
    vf.create_multiObj_vehicle_std_shade_plot(result_path, config)
    vf.create_multiObj_vehicle_boxplot_double(result_path, config)
    vf.create_multiObj_vehicle_heatmap(result_path, config)
    """

""" customer_weight evaluation"""
def plot_customer_weight(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_custW3_barplot(result_path, config)
    vf.create_custW3_time_boxplot_three(result_path, config)
    """
    vf.create_custW3_wait_grouped(result_path, config)
    vf.create_custW3_waitcust_grouped(result_path, config)
    vf.create_custW3_time_boxplot_double(result_path, config)
    vf.create_custW3_time_boxplot_forth(result_path, config)
    """


""" compare evaluation"""
def plot_compare(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    isud_path = c.OUTPUT_DIR + f"{phase}/{c.FOLDERS[phase][selected_folder]}" + "/" + "ISUD_results.csv"

    vf.create_compare_isud_barplot_single(result_path, isud_path, config)
    vf.create_compare_isud_boxplot_double(result_path, isud_path, config)

    vf.create_compare_barplot_single(result_path, isud_path, config)
    vf.create_compare_request_boxplot_triple(result_path, isud_path, config)
    """
    vf.create_compare_wait_grouped(result_path, config)
    vf.create_compare_request_boxplot_double(result_path, config)
    vf.create_compare_time_boxplot_double(result_path, config)
    """

""" obj formulation evaluation"""
def plot_obj_compare(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_obj_compare_request_boxplot_triple(result_path, config)
    vf.create_obj_compare_request_boxplot_fourth(result_path, config)
    vf.create_obj_compare_multi_std_plot(result_path, config)
    vf.create_obj_compare_time_request_profile(result_path, config)
    vf.create_obj_compare_vehicle_KPI_boxplot_double(result_path, config)

# --- mapping from folder name to plotting function ---
PLOT_FUNCTIONS = {
    "nbPicks": plot_nbPick,
    "pruning": plot_pruning,
    "truncate": plot_truncated,
    "dropPick": plot_dropPick,
    "dynamic": plot_dynamic,
    "commit": plot_commit,
    "ablation": plot_ablation,
    "multiObj": plot_multiObj,
    "customer_weight": plot_customer_weight,
    "compare": plot_compare,
    "obj_compare": plot_obj_compare,
}


def main(
    name: str,
    phase: str = "Phase_2",
    selected_folders: Union[str, Iterable[str]] = "compare",
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
            merge_with_averaged_epoch(result_folder)
     #       merge_with_all_iterations(result_folder)

        print(f"[INFO] Running plots for folder '{folder}' in phase '{phase}'")
        plot_fn(phase=phase, selected_folder=folder, config=config)


if __name__ == "__main__":
    # Use scripts/plot_BCG.py for the CLI entry point:
    #   python scripts/plot_BCG.py --folders multiObj
    #   python scripts/plot_BCG.py --folders all
    main("", selected_folders="multiObj", gather_data=False)
