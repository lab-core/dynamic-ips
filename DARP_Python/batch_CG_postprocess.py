import constants as c
import Visualization.visualize_batch_CG as vf
import Visualization.visualize_batch_custom as vc
from ProcessResults.merge_records import merge_with_averaged_epoch
from Visualization.plot_config import PlotConfig


def setup_paths(phase: str, selected_folder: str) -> tuple[str, str]:
    """Setup folder and result paths."""
    folder_name = c.FOLDERS[phase][selected_folder]
    result_folder = f"{phase}/{folder_name}"
    result_path = c.OUTPUT_DIR + result_folder + "/" + "results_epoch_avg.csv"
    return result_folder, result_path

""" Pickup Limit"""
def plot_nbPick(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_nbPick_time_barplot_double(result_path, config)
    vf.create_nbPick_wait_barplot_double(result_path, config)
    vf.create_nbPick_obj_barplot_double(result_path, config)

""" Pruning Strategy"""
def plot_pruning(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_pruning_time_boxplot_double(result_path, config)
    vf.create_pruning_scatter_plot_double(result_path, config)

""" Truncated Labeling"""
def plot_truncated(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_truncate_time_boxplot(result_path, config)
    vc.create_truncate_objective_scatter(result_path, config)

""" DropPick """
def plot_dropPick(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_dropPick_time_boxplot_double(result_path, config)
    vf.create_dropPick_wait_barplot_double(result_path, config)

""" Dynamic Pickups"""
def plot_dynamic(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_dynamic_time_boxplot_double(result_path, config)
    vf.create_dynamic_avgtime_boxplot_double(result_path, config)
    vc.create_dynamic_iteration_bubble_plot_double(result_path, config)

""" Commit evaluation"""
def plot_commit(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_commit_wait_boxplot_double(result_path, config)
    vf.create_commit_response_boxplot_double(result_path, config)

""" Ablation evaluation"""
def plot_ablation(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_ablation_wait_boxplot_double(result_path, config)
    vf.create_ablation_time_boxplot_double(result_path, config)

""" multiObj evaluation"""
def plot_multiObj(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    """
    vf.create_multiObj_request_boxplot_double(result_path, config)
    vf.create_multiObj_vehicle_boxplot_double(result_path, config)
    vf.create_multiObj_time_boxplot_double(result_path, config)
    vf.create_multiObj_wait_grouped(result_path, config)
    vf.create_multiObj_tripdelay_grouped(result_path, config)
    vf.create_multiObj_passenger_grouped(result_path, config)
    vf.create_multiObj_idle_grouped(result_path, config)
    """
    vf.create_multiObj_time_request_profile(result_path, config)
    vf.create_multiObj_passenger_profile(result_path, config)

""" customer_weight evaluation"""
def plot_customer_weight(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)
    vf.create_custW3_wait_grouped(result_path, config)
    vf.create_custW3_waitcust_grouped(result_path, config)
    vf.create_custW3_time_boxplot_double(result_path, config)

""" compare evaluation"""
def plot_compare(phase: str, selected_folder: str, config: PlotConfig) -> None:
    result_folder, result_path = setup_paths(phase, selected_folder)

def main(name):
    gather_data = False
    phase = 'Phase_2'
    selected_folder = 'ablation'

    # Get the full path:
    result_folder, result_path = setup_paths(phase, selected_folder)
    config = PlotConfig()

    if gather_data:
        merge_with_averaged_epoch(result_folder)

    """
    plot_nbPick(phase=phase, selected_folder='nbPicks', config=config)
    plot_pruning(phase=phase, selected_folder='pruning', config=config)
    plot_truncated(phase=phase, selected_folder='truncate', config=config)
    plot_dropPick(phase=phase, selected_folder='dropPick', config=config)
    plot_dynamic(phase=phase, selected_folder='dynamic', config=config)
    plot_commit(phase=phase, selected_folder='commit', config=config)
    plot_ablation(phase=phase, selected_folder='ablation', config=config)
    plot_customer_weight(phase=phase, selected_folder='customer_weight', config=config)
    plot_multiObj(phase=phase, selected_folder='multiObj', config=config)
    """

    plot_compare(phase=phase, selected_folder='compare', config=config)
    plot_multiObj(phase=phase, selected_folder='multiObj', config=config)



if __name__ == '__main__':
    main('PyCharm')
