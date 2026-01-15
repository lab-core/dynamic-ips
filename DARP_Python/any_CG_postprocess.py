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



""" rebalance evaluation"""
def plot_rebalance(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)

""" partial evaluation"""
def plot_partial(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)

""" prcocess evaluation"""
def plot_prcocess(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)

""" MP_method evaluation"""
def plot_MP_method(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)

""" compare evaluation"""
def plot_compare(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    isud_path = c.OUTPUT_DIR + f"{phase}/{c.FOLDERS[phase][selected_folder]}" + "/" + "ISUD_results.csv"



# --- mapping from folder name to plotting function ---
PLOT_FUNCTIONS = {
    "rebalance": plot_rebalance,
    "partial": plot_partial,
    "prcocess": plot_prcocess,
    "MP_method": plot_MP_method,
    "compare": plot_compare,
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
            merge_with_averaged_epoch(result_folder)
        #       merge_with_all_iterations(result_folder)

        print(f"[INFO] Running plots for folder '{folder}' in phase '{phase}'")
        plot_fn(phase=phase, selected_folder=folder, config=config)


if __name__ == "__main__":
    # Example: only run ablation plots
    main("PyCharm", selected_folders="rebalance", gather_data=True)
    # Example: run several
    # main("PyCharm", selected_folders=["ablation", "multiObj"])
    # Example: run everything
    # main("PyCharm", selected_folders="all")
