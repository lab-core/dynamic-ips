import os
import constants as c
import seaborn as sns

from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.data_preprocessing import preprocess_nested_data, build_vehicle_kpi_long_df
from Visualization.plot_config import PlotConfig
from Visualization.visualize_bar_plots import create_double_relative_barplot, create_double_grouped_barplot, \
    _plot_grouped_bars, prepare_grouped_data
from Visualization.visualize_box_plots import create_comparison_boxplots, create_single_boxplot, \
    create_multi_subplot_boxplots, plot_epoch_vehicle_boxplot
from Visualization.visualize_others import create_single_heatmap_figure, create_shaded_mean_std_plot
from Visualization.visualize_profile_plots import create_multi_subplot_profile_lineplots
from Visualization.visualize_scatter_plots import plot_pruning_scatter_double, create_multi_subplot_lineplots, \
    create_grouped_lineplot, plot_pruning_scatter_single
from Visualization.visualize_violin_plots import create_multi_subplot_violinplots

outlier = True


# ---------------------------------------------------------------------------
# Box plots
# ---------------------------------------------------------------------------
def create_pruning_time_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for pruning strategies comparison."""

    return create_comparison_boxplots(
        data_path=data_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='isSuccessorsLimited',
        value_column='Total time',
        categories=['noPruning', 'pruneNodes', 'pruneArcs', 'discardSuboptimalPath'],
        category_labels=['No pruning', 'Prune nodes', 'Prune arcs', 'Prune suboptimal path'],
        legend_title='Pruning Strategies',
        ylabel='Epoch Runtime (s)',
        output_filename='pruning_time_boxplot.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        target_lines=30,
        rotation=15,
        ylim=[1150, 750],  # Will apply to both subplots
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.42, 0.865),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=True,
        fig_size=config.fig_size,
        tight_layout_rect=(0, 0, 1, 0.8)
    )

def create_pruning_time_boxplot_single(data_path: str, config: PlotConfig):
    """Single subplot showing runtime for different sorting strategies."""

    # Define filtering function
    def preprocess_pruning(df):
        df = df[df['Ride_W2'] == 0]
        return df

    subplot_configs = [
        {
            'item_column': 'Instance',
            'category_column': 'isSuccessorsLimited',
            'value_column': 'Total time',
            'categories':['noPruning', 'pruneNodes', 'pruneArcs', 'discardSuboptimalPath'],
            'category_labels':['No pruning', 'Prune nodes', 'Prune arcs', 'Prune suboptimal path'],
            'ylabel': 'Runtime (s)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.35,
            'show_outliers': outlier,
            'target_lines': 30,
            'show_legend': False,  # we will use a shared legend
            'ylim' : [1150, 750],
        }
    ]
    shared_legend_config = {
        'loc': 'upper center',
        'bbox' : (0.55, 0.99),  # bbox_anchor_legend1
        'ncol': 1,
        'title': 'Pruning Strategies'
    }
    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.5, 0.775)
    }

    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs,
        output_filename='pruning_time_boxplot_single.pdf',
        fig_size=(3, 4),
        n_rows=1,
        n_cols=1,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=preprocess_pruning,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.81),
    )

def create_dropPick_time_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for drop-pick options comparison."""

    return create_comparison_boxplots(
        data_path=data_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='isDropPickPossible',
        value_column='Total time',
        categories=['True', 'False'],
        category_labels=['Drop-Pick Allowed', 'Drop-Pick not Allowed'],
        legend_title='Drop-Pick Options',
        ylabel='Total Runtime (s)',
        output_filename='dropPick_time_boxplot.pdf',
        width=0.55,
        target_lines=30,
        rotation=15,
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.4, 0.89),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=True,
        fig_size=config.fig_size,
        tight_layout_rect=(0, 0, 1, 0.85)
    )


def create_dropPick_wait_time_baxplot_double(data_path: str, config: PlotConfig) -> str:
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df['Ride_W2'] == 0]
    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {
            # LEFT: Problem Size
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'isDropPickPossible',
            'value_column': 'EpochRuntime',
            'categories': ['True', 'False'],
            'category_labels': ['Drop-Pick allowed', 'Drop-Pick not allowed'],
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # RIGHT: Epoch Runtime
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'isDropPickPossible',
            'value_column': 'wait/cust',
            'categories': ['True', 'False'],
            'category_labels': ['Drop-Pick allowed', 'Drop-Pick not allowed'],
            'ylabel': 'Wait Time per Request (s)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: #pass Per Vehicle
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'isDropPickPossible',
            'value_column': '#passPerVehicle',
            'categories': ['True', 'False'],
            'category_labels': ['Drop-Pick allowed', 'Drop-Pick not allowed'],
            'ylabel': '# Passengers',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Requests Per Vehicle
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'isDropPickPossible',
            'value_column': '#requestPerVehicle',
            'categories': ['True', 'False'],
            'category_labels': ['Drop-Pick allowed', 'Drop-Pick not allowed'],
            'ylabel': '# Request',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 2,
        'title': 'Drop-Pick Options'
    }
    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.35, 0.9)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename='dropPick_wait_time_boxplot.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.88],
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename='dropPick_request_pass_boxplot.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.9],
    )

    return figure_path

def create_dynamic_wait_time_baxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df['Ride_W2'] == 0]

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {
            # LEFT: Problem Size
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'Dynamic_Pricing',
            'value_column': 'EpochRuntime',
            'categories': ['False', 'True'],
            'category_labels': ['2 Pickups', 'Dynamic Pickups Limit'],
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # RIGHT: Epoch Runtime
            'data_df': main_df,
            'item_column': 'Instance',
            'category_column': 'Dynamic_Pricing',
            'value_column': 'wait/cust',
            'categories': ['False', 'True'],
            'category_labels': ['2 Pickups', 'Dynamic Pickups Limit'],
            'ylabel': 'Wait Time per Request (s)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.55,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 2,
        'title': 'Pickup Limits'
    }
    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.38, 0.923)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename='dynamic_wait_time_boxplot.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.9],
    )

    return figure_path


def create_dynamic_time_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for dynamic pricing comparison with drop-pick filter."""

    return create_comparison_boxplots(
        data_path=data_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='Dynamic_Pricing',
        value_column='Total time',
        categories=['False', 'True'],
        category_labels=['2 Pickups', 'Dynamic Pickups Limit'],
        legend_title='Pickup Limits',
        ylabel='Total Runtime (s)',
        output_filename=f'dynamic_time_boxplot.pdf',
        width=0.55,
        target_lines=30,
        rotation=15,
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.43, 0.89),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=True,
        fig_size=config.fig_size,
        tight_layout_rect=(0, 0, 1, 0.85)
    )


def create_dynamic_avgtime_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for dynamic average time comparison."""

    # Preprocess function to calculate average time
    def preprocess_avgtime(df):
        df['avg time'] = df['Total time'] / df['#SP Iter ']
        return df

    return create_comparison_boxplots(
        data_path=data_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='Dynamic_Pricing',
        value_column='avg time',
        categories=['False', 'True'],
        category_labels=['2 Pickups', 'Dynamic Pickups Limit'],
        legend_title='Pickup Limits',
        ylabel='Total Runtime (s)',
        output_filename=f'dynamic_avgtime_boxplot.pdf',
        width=0.55,
        target_lines=30,
        rotation=15,
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.43, 0.89),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=True,
        fig_size=config.fig_size,
        additional_filter=preprocess_avgtime,
        tight_layout_rect=(0, 0, 1, 0.85)
    )


def create_truncate_time_boxplot(data_path: str, config: PlotConfig):
    """Single subplot showing runtime for different sorting strategies."""

    # Define filtering function
    def preprocess_truncate(df):
        df = df[(df['Ride_W2'] == 0) & (df['isTruncated'] == True)]
        return df

    return create_single_boxplot(
        data_path=data_path,
        config=config,
        item_column='MaxLabel',  # X-axis groups by MaxLabel
        category_column='sortPath',
        value_column='Total time',
        categories=['LAMBDA_SCORE', 'PATH_SCORE', 'REDUCED_COST'],
        category_labels=['Lambda score', 'Normalized reduced cost', 'Reduced cost'],
        legend_title='Sort Options',
        ylabel='Epoch Runtime (s)',
        xlabel='$M^{Label}$',
        output_filename='truncate_boxplot.pdf',
        color_reverse=False,
        width=0.5,
        target_lines={'Epoch size (30s)': 30},
        rotation=0,
        additional_filter=preprocess_truncate,
        show_outliers=True,
        ylim = (0 , 530),
        legend_loc = 'upper left',
    )


def create_commit_response_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess commit wait time data
    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column='CommitWaitTime',
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],  # Filter unrealistic values
        transform_func=lambda x: x / 60,  # Convert to minutes
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'Algorithm_Name']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Create comparison boxplot
    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='Algorithm_Name',
        value_column='CommitWaitTime',
        categories=['B-CG', 'B-CG-C'],
        #       category_labels=['B-CG', 'B-CG-C'],
        legend_title='Algorithms',
        ylabel='Response Times (min)',
        output_filename=f'commit_responseTime_boxplot_{outlier}.pdf',
        width=0.5,
        rotation=15,
        legend_bbox=(0.5, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.88),
        fig_size=config.fig_size
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path


def create_commit_wait_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess commit wait time data
    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column='WaitTime',
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],  # Filter unrealistic values
        transform_func=lambda x: x / 60,  # Convert to minutes
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'Algorithm_Name']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Create comparison boxplot
    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance',
        category_column='Algorithm_Name',
        value_column='WaitTime',
        categories=['B-CG', 'B-CG-C'],
        #       category_labels=['B-CG', 'B-CG-C'],
        legend_title='Algorithms',
        ylabel='Waiting Times (min)',
        output_filename=f'commit_waitTime_boxplot_{outlier}.pdf',
        width=0.5,
        rotation=15,
        legend_bbox=(0.5, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.88),
        fig_size=config.fig_size
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_commit_wait_response_baxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'CommitWaitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Ride_W2', 'Algorithm_Name']
    )
    processed_df = processed_df[processed_df['Ride_W2'] == 0]

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Waiting Times
            'item_column': 'Instance',
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': ['B-CG', 'B-CG-C'],
            'category_labels': ['B-CG', 'B-CG-C'],
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
    #        'ylim': (-2, 110),
        },
        {  # RIGHT: Response Times
            'item_column': 'Instance',
            'category_column': 'Algorithm_Name',
            'value_column': 'CommitWaitTime',
            'categories': ['B-CG', 'B-CG-C'],
            'category_labels': ['B-CG', 'B-CG-C'],
            'ylabel': 'Response Times (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
    #        'ylim' : (-2, 110),
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'commit_response_wait_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_commit_wait_response_baxplot_group(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'CommitWaitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Ride_W2', 'Algorithm_Name', 'Instance_category']
    )
    processed_df = processed_df[processed_df['Ride_W2'] == 0]

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Waiting Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': ['B-CG', 'B-CG-C'],
            'category_labels': ['B-CG', 'B-CG-C'],
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
 #           'ylim': (0, 99),
            'ylim': (0, 16.8),
        },
        {  # RIGHT: Response Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'CommitWaitTime',
            'categories': ['B-CG', 'B-CG-C'],
            'category_labels': ['B-CG', 'B-CG-C'],
            'ylabel': 'Response Times (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
 #           'ylim' : (0, 98),
            'ylim': (0, 16.8),
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'commit_response_wait_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path


def create_ablation_wait_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
 #   main_df = main_df[main_df['Dynamic_Pricing'] == True]

    # Preprocess commit wait time data
    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],  # Filter unrealistic values
        transform_func=lambda x: x / 60,  # Convert to minutes
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Create comparison boxplot
    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='WaitTime',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='Waiting Times (min)',
        output_filename=f'ablation_waitTime_boxplot_{outlier}.pdf',
        width=0.35,
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='TripDelay',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='Trip Delay (min)',
        output_filename=f'ablation_tripDelay_boxplot_{outlier}.pdf',
        width=0.35,
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    # Clean up
 #   if os.path.exists(temp_path):
 #       os.remove(temp_path)

    return figure_path


def create_ablation_time_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
   # main_df = main_df[main_df['Dynamic_Pricing'] == True]
    # Preprocess commit wait time data

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', 'Epoch',
                      '#nodePerVehicle', '#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Create comparison boxplot
    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='EpochRuntime',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='Epoch Runtime (s)',
        output_filename=f'ablation_runTime_boxplot_{outlier}.pdf',
        target_lines=30,
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.42, 0.865),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.8),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='nbRequests',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='# Requests per Epoch',
        output_filename=f'ablation_request_boxplot_{outlier}.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='#passPerVehicle',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='Initial Vehicle Load',
        output_filename=f'ablation_onboards_boxplot_{outlier}.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='#requestPerVehicle',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='# Request per Vehicle',
        output_filename=f'ablation_req_boxplot_{outlier}.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='#nodePerVehicle',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='# Stops per Vehicle',
        output_filename=f'ablation_stop_boxplot_{outlier}.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    figure_path = create_comparison_boxplots(
        data_path=temp_path,
        config=config,
        comparison_column='Ride_W2',
        comparison_values=[0, 0.5],
        item_column='Instance_category',
        category_column='setting',
        value_column='#SP Iter',
        categories=c.param_to_setting_labels,
        category_labels=c.param_to_setting_labels,
        legend_title='Setting',
        ylabel='# Iterations per Epoch',
        output_filename=f'ablation_iter_boxplot_{outlier}.pdf',
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        rotation=15,
        legend_bbox=(0.55, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    # Clean up
 #   if os.path.exists(temp_path):
 #       os.remove(temp_path)

    return figure_path


def create_multiObj_request_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Waiting Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'WaitTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Trip delay
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Trip delay',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: Response Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'CommitWaitTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Response Times (min)',
            'xlabel': 'Response Times',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Assign Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'AssignTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Assign Time (min)',
            'xlabel': 'Assign Time',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'multiObj_wait_delay_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'multiObj_response_assign_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_multiObj_request_boxplot_triple(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_request_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )
    processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

    # Save to temporary file
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_request_df.to_csv(request_path, index=False)

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    # Save to temporary file
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_time_df.to_csv(time_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'WaitTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0,28),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'nbRequests',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 2800),
            'show_outliers': outlier,
            'show_legend': False,
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'multiObj_wait_delay_request_boxplot_{outlier}.pdf',
        fig_size=(6, 4),
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
    )

    # Clean up
    if os.path.exists(request_path):
        os.remove(request_path)
        os.remove(time_path)

    return figure_path

def create_multiObj_vehicle_KPI_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)
    KPI_columns_left = ['idleTime', 'driveFullTime']
    kpi_name_map_left = {
        'idleTime': 'Idle Time',
        'driveFullTime': 'Drive Full',
    }

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='vehicle',
        value_column=['idleTime', 'driveEmptyTime', 'driveFullTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    df_left = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=KPI_columns_left,
        kpi_name_map=kpi_name_map_left,
        category_column='object_category',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )
    df_right = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=['driveEmptyTime'],
        kpi_name_map={'driveEmptyTime': 'Drive Empty'},
        category_column='object_category',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Idle Times
            "data_df": df_left,
            'item_column': 'KPI',
            'category_column': 'object_category',
            'value_column': 'value',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Times (min)',
            'xlabel': '       Vehicle KPIs',
            'rotation': 15,
            'ylim': (0, 125),
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Drive Empty
            "data_df": df_right,
            'item_column': 'KPI',
            'category_column': 'object_category',
            'value_column': 'value',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Times (min)',
            'xlabel': ' ',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 24),
            'show_outliers': outlier,
            'show_ylabel': False,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'multiObj_driving_KPI_boxplot_{outlier}.pdf',
        fig_size=(3,4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0.02, 1, 0.9),
    )

    return figure_path


def create_multiObj_vehicle_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='vehicle',
        value_column=['idleTime', 'driveFullTime', 'driveEmptyTime', 'TripDelay'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'vehicle_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Idle Times
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'idleTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Idle Times (min)',
            'xlabel': 'Idle Times',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Drive Empty
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'driveEmptyTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Drive Empty (min)',
            'xlabel': 'Drive Empty',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: Drive full
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'driveFullTime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Drive full Times (min)',
            'xlabel': 'Drive full Times',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Requests Served
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Excess Ride time (min)',
            'xlabel': 'Excess Ride time',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'multiObj_idle_empty_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'multiObj_full_delay_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_multiObj_vehicle_KPI_boxplot(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)
    KPI_columns_left = ['idleTime', 'driveFullTime']
    kpi_name_map_left = {
        'idleTime': 'Idle Time',
        'driveFullTime': 'Drive Full',
    }


    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='vehicle',
        value_column=['idleTime', 'driveEmptyTime', 'driveFullTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    df_left = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=KPI_columns_left,
        kpi_name_map=kpi_name_map_left,
        category_column='object_category',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )
    df_right = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=['driveEmptyTime'],
        kpi_name_map={'driveEmptyTime': 'Drive Empty'},
        category_column='object_category',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Idle Times
            "data_df": df_left,
            'item_column': 'KPI',
            'category_column': 'object_category',
            'value_column': 'value',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Times (min)',
            'xlabel': 'Vehicle KPIs',
            'rotation': 0,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Drive Empty
            "data_df": df_right,
            'item_column': 'KPI',
            'category_column': 'object_category',
            'value_column': 'value',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Times (min)',
            'xlabel': 'Vehicle KPIs',
            'rotation': 0,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'driving_KPI_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    return figure_path

def create_multiObj_time_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'nbRequests',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': '# Requests',
            'xlabel': 'Problem size per Epoch',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'EpochRuntime',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Runtime per Epoch',
            'rotation': 20,
            'width': 0.4,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: #pass Per Vehicle
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': '#passPerVehicle',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': '# Passengers',
            'xlabel': 'Passengers Per Vehicle',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Requests Per Vehicle
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': '#requestPerVehicle',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': '# Request',
            'xlabel': 'Requests Per Vehicle',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'multiObj_size_time_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.9],
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'multiObj_pass_req_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.9],
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path


def create_custW3_time_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#LGenerated', 'EpochRuntime', 'totalRoutes', 'meanDual'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'Req_W3']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]
    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': '#LGenerated',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated labels',
            'xlabel': 'Generated labels per Epoch',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 24000000),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'EpochRuntime',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Runtime per Epoch',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 34),
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: Problem Size
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'totalRoutes',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated routes',
            'xlabel': 'Generated routes per Epoch',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 680000),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'meanDual',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Mean dual value',
            'xlabel': 'Mean dual value',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'custW3_lable_time_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'custW3_route_dual_boxplot_{outlier}.pdf',
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_custW3_time_boxplot_forth(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#LGenerated', 'EpochRuntime', 'totalRoutes', 'meanDual'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'Req_W3']
    )

    processed_request_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'Req_W3']
    )


    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]
    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # RIGHT: Epoch Runtime
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'EpochRuntime',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 39),
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # LEFT: Problem Size
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': '#LGenerated',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated labels',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 130000000),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # LEFT: Problem Size
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'WaitTime',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
 #           'ylim': (0, 1900000),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'meanDual',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Mean dual value',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.98),
        'ncol': 2,
        'title': 'Object Settings'
    }

    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.408, 0.89)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'custW3_boxplot_{outlier}.pdf',
        fig_size=(13, 4),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend = shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    return figure_path

def create_custW3_time_boxplot_three(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#LGenerated', 'EpochRuntime', 'totalRoutes', 'meanDual'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'Req_W3']
    )

    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]
    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # RIGHT: Epoch Runtime
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'EpochRuntime',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 39),
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # RIGHT: Epoch Runtime
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': 'meanDual',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': 'Mean dual value',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # LEFT: Problem Size
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Req_W3',
            'value_column': '#LGenerated',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated labels',
            'xlabel': 'Customer Group',
            'rotation': 15,
            'width': 0.4,
            'ylim': (0, 130000000),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.35, 0.98),
        'ncol': 2,
        'title': 'Object Settings'
    }

    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.21, 0.88)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'custW3_boxplot3_{outlier}.pdf',
        fig_size=(9, 4),
        n_rows=1,
        n_cols=3,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend = shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.9),
    )

    return figure_path

def create_compare_request_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['object_category', 'customer Group']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Waiting Times
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'WaitTime',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Trip delay
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Trip delay',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: Response Times
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'CommitWaitTime',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': 'Response Times (min)',
            'xlabel': 'Response Times',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Assign Times
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'AssignTime',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': 'Assign Time (min)',
            'xlabel': 'Assign Time',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'compare_wait_delay_boxplot_{outlier}.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.88],
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'compare_response_assign_boxplot_{outlier}.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.88],
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_compare_time_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'customer Group']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'nbRequests',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': '# Requests',
            'xlabel': 'Problem size per Epoch',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': 'EpochRuntime',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Runtime per Epoch',
            'rotation': 15,
            'width': 0.4,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: #pass Per Vehicle
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': '#passPerVehicle',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': '# Passengers',
            'xlabel': 'Passengers Per Vehicle',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Requests Per Vehicle
            'item_column': 'customer Group',
            'category_column': 'object_category',
            'value_column': '#requestPerVehicle',
            'categories': c.OBJECTIVES[0:2],
            'category_labels': c.OBJECTIVES_labels[0:2],
            'ylabel': '# Request',
            'xlabel': 'Requests Per Vehicle',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'compare_size_time_boxplot_{outlier}.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.88],
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'compare_pass_req_boxplot_{outlier}.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0, 0, 1, 0.88],
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path

def create_epoch_vehicle_boxplot(data_path: str, config: PlotConfig) -> None:
    # Read summary config CSV
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df['Instance_category'] == '4.High Demand']
    base_path = os.path.dirname(data_path)

    # Output folder
    main_folder_path = os.path.join(base_path, f'vehicle_epoch_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    folder_path = os.path.join(main_folder_path, 'vehicle_boxplots_length')
    os.makedirs(folder_path, exist_ok=True)

    def build_file_path(row):
        mode = f"{c.MODE_DICT.get(row['Mode'])}_{row['Algorithm']}"
        file_name = c.DATA_TYPE_FILES['epochVehicle'].format(mode=mode)
        return os.path.join(base_path, row['Test_Folder'], file_name)

    # Loop over test rows
    for _, row in main_df.iterrows():
        file_path = build_file_path(row)
        results_df = read_csv_with_encoding(file_path)

        output_path = os.path.join(
            folder_path,
            f"{row['Instance']}_{row['Ride_W2']}_boxplot.pdf"
        )

        plot_epoch_vehicle_boxplot(
            results_df=results_df,
            output_path=output_path,
            config=config,
            value_column='length',
            ylabel = '# Length',
            ylim = 2000,
        )

        print(f"Saved vehicle boxplot → {output_path}")

def create_compare_request_boxplot_triple(data_path: str, data_path_ISUD: str, config: PlotConfig) -> str:
    import pandas as pd
    # -----------------------------
    # 1. Read data from both sources
    # -----------------------------
    df_main = read_csv_with_encoding(data_path)
    df_isud = read_csv_with_encoding(data_path_ISUD)
    palette = sns.color_palette("gist_earth", n_colors=4)[0:3][::-1]
    df_isud['Test_Folder'] = df_isud['Name'] + "/" + df_isud['Test_Folder']

    # -----------------------------
    # 3. Helper: encode W_delay into Algorithm_Name
    # -----------------------------
    def _add_alg_with_delay(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()

        def _alg_with_delay(row):
            if row["Ride_W2"] == 0:
                suffix = r" $(\omega_{drop}$ = 0)"
            elif row["Ride_W2"] == 0.5:
                suffix = r" $(\omega_{drop}$ = 0.5)"
            else:
                # fallback for any unexpected value
                suffix = rf" $(W_{{delay}}$ = {row['Ride_W2']})"
            return f"{row['Algorithm_Name']}{suffix}"

        df["Algorithm_Name"] = df.apply(_alg_with_delay, axis=1)
        return df

    df_main = _add_alg_with_delay(df_main)
    df_isud = _add_alg_with_delay(df_isud)

    df = pd.concat([df_main, df_isud], ignore_index=True)

    preferred_algo_order = [
        r"F-ICG $(\omega_{drop}$ = 0)",
        r"B-CG-C $(\omega_{drop}$ = 0)",
        r"B-CG-C $(\omega_{drop}$ = 0.5)",
    ]
    unique_algos = list(df["Algorithm_Name"].unique())
    category_keys = (
            [a for a in preferred_algo_order if a in unique_algos] +
            [a for a in unique_algos if a not in preferred_algo_order]
    )
    categories = {k: k for k in category_keys}
    category_labels = list(categories.values())

    # Preprocess data (same as before)

    processed_request_df = preprocess_nested_data(
        df=df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Algorithm_Name', 'customer Group']
    )
    processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

    # Save to temporary file
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_request_df.to_csv(request_path, index=False)

    processed_time_df = preprocess_nested_data(
        df=df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Algorithm_Name', 'customer Group']
    )

    # Save to temporary file
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_time_df.to_csv(time_path, index=False)


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'customer Group',
            'x_tick_labels': c.customer_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': categories,
            'category_labels': category_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': ' ',
            'rotation': 0,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            "ylim": (0, 44),
        },
        {  # 2: Trip delay
  #          'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'customer Group',
            'x_tick_labels': c.customer_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'TripDelay',
            'categories': categories,
            'category_labels': category_labels,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Customer Group',
            'rotation': 0,
            'width': 0.44,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
            "ylim": (0, 9.5),
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df,
            'item_column': 'customer Group',
            'x_tick_labels': c.customer_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': categories,
            'category_labels': category_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': ' ',
            'rotation': 0,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,
            'target_lines' : 30,
            "ylim": (0, 32),
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.67, 0.99),
        'ncol': 3,
        'title': 'Algorithms'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'compare_wait_delay_request_boxplot_{outlier}.pdf',
        fig_size=(10, 4),
        n_rows=1,
        n_cols=3,
        shared_legend=False,
 #       shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        palette = palette,
 #       tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
    )

    # Clean up
    if os.path.exists(request_path):
        os.remove(request_path)
     #   os.remove(time_path)

    return figure_path

def create_compare_isud_boxplot_double(data_path: str, data_path_ISUD: str, config: PlotConfig) -> str:
    import pandas as pd
    # -----------------------------
    # 1. Read data from both sources
    # -----------------------------
    df_isud = read_csv_with_encoding(data_path_ISUD)
    palette = sns.color_palette("gist_earth", n_colors=4)[0:3][::-1]
    df_isud['Test_Folder'] = df_isud['Name'] + "/" + df_isud['Test_Folder']

    # -----------------------------
    # 3. Helper: encode W_delay into Algorithm_Name
    # -----------------------------
    def _add_alg_with_delay(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()

        def _alg_with_delay(row):
            if row["Ride_W2"] == 0:
                suffix = r" $(\omega_{drop}$ = 0)"
            elif row["Ride_W2"] == 0.5:
                suffix = r" $(\omega_{drop}$ = 0.5)"
            else:
                # fallback for any unexpected value
                suffix = rf" $(W_{{delay}}$ = {row['Ride_W2']})"
            return f"{row['Algorithm_Name']}"

        df["Algorithm_Name"] = df.apply(_alg_with_delay, axis=1)
        return df

    df_isud = _add_alg_with_delay(df_isud)

    df = pd.concat([df_isud], ignore_index=True)

    preferred_algo_order = [
        r"F-IC $(\omega_{drop}$ = 0)",
    ]
    unique_algos = list(df["Algorithm_Name"].unique())
    category_keys = (
            [a for a in preferred_algo_order if a in unique_algos] +
            [a for a in unique_algos if a not in preferred_algo_order]
    )
    categories = {k: k for k in category_keys}
    category_labels = list(categories.values())

    # Preprocess data (same as before)

    processed_request_df = preprocess_nested_data(
        df=df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Algorithm_Name', 'customer Group']
    )
    processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

    # Save to temporary file
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    processed_request_df.to_csv(request_path, index=False)

    processed_time_df = preprocess_nested_data(
        df=df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Algorithm_Name', 'customer Group']
    )

    # Save to temporary file
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_time_df.to_csv(time_path, index=False)


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'customer Group',
            'x_tick_labels': c.customer_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': categories,
            'category_labels': category_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': ' ',
            'rotation': 0,
            'width': 0.8,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            "ylim": (0, 44),
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df,
            'item_column': 'customer Group',
            'x_tick_labels': c.customer_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': categories,
            'category_labels': category_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': ' ',
            'rotation': 0,
            'width': 0.8,
            'show_outliers': outlier,
            'show_legend': False,
            'target_lines' : 30,
            "ylim": (0, 32),
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.67, 0.99),
        'ncol': 3,
        'title': 'Algorithms'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'compare_wait_delay_request_boxplot_{outlier}.pdf',
        fig_size=(4.5, 4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
 #       shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        palette = palette,
 #       tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
    )

    # Clean up
    if os.path.exists(request_path):
        os.remove(request_path)
     #   os.remove(time_path)

    return figure_path

# ---------------------------------------------------------------------------
# Bar plots
# ---------------------------------------------------------------------------
def create_nbPick_wait_barplot_double(data_path: str, config: PlotConfig):
    return create_double_grouped_barplot(
        data_path=data_path,
        config=config,
        group_columns=['Instance', 'nbPick'],
        agg_column='wait/req',
        categories={1: '1 Pickup', 2: '2 Pickups', 3: '3 Pickups', 4: '4 Pickups'},
        category_labels=['1 Pickup', '2 Pickups', '3 Pickups', '4 Pickups'],
        legend_title='Pickup Limits',
        ylabel='Average Waiting time (s)',
        output_filename='nbPick_wait_barplot.pdf',
        ride_w2_values=[0, 0.5],
        color_reverse=True,
        width=0.1,
        cluster_spacing=2.5,
        rotation=15,
        log_scale=False,
        ylim_left=None,
        ylim_right=None,
        target_line=None,
        bbox_anchor_legend1=(0.5, 0.99),
        alpha=0.7,
        agg_func='mean'
    )

def create_nbPick_obj_barplot_double(data_path: str, config: PlotConfig):
    return create_double_grouped_barplot(
        data_path=data_path,
        config=config,
        group_columns=['Instance', 'nbPick'],
        agg_column='AvgObjective',
        categories={1: '1 Pickup', 2: '2 Pickups', 3: '3 Pickups', 4: '4 Pickups'},
        category_labels=['1 Pickup', '2 Pickups', '3 Pickups', '4 Pickups'],
        legend_title='Pickup Limits',
        ylabel='Avg. Objective / request (s)',
        output_filename='nbPick_obj_barplot.pdf',
        ride_w2_values=[0, 0.5],
        color_reverse=True,
        width=0.1,
        cluster_spacing=2.5,
        rotation=15,
        bbox_anchor_legend1=(0.5, 0.99),
        alpha=0.7
    )

def create_nbPick_time_barplot_double(data_path: str, config: PlotConfig):
    return create_double_grouped_barplot(
        data_path=data_path,
        config=config,
        group_columns=['Instance', 'nbPick'],
        agg_column='Total time',
        categories={1: '1 Pickup', 2: '2 Pickups', 3: '3 Pickups', 4: '4 Pickups'},
        category_labels=['1 Pickup', '2 Pickups', '3 Pickups', '4 Pickups'],
        legend_title='Pickup Limits',
        ylabel='Total Runtime (s)',
        output_filename='nbPick_time_barplot.pdf',
        ride_w2_values=[0, 0.5],
        color_reverse=True,
        width=0.1,
        cluster_spacing=2.5,
        rotation=15,
        log_scale=True,  # Use log scale
        target_line=30,  # Add epoch size line
        target_line_label='Epoch size (30s)',
        bbox_anchor_legend1=(0.55, 0.99),
        bbox_anchor_legend2=(0.495, 0.86),
        alpha=0.7
    )

def create_nbPick_barplot_double(data_path: str, config: PlotConfig) -> str:
    import matplotlib.lines as mlines
    from matplotlib import pyplot as plt
    """
    Create a 1x2 figure:
      - Left: AvgObjective grouped by (Instance, nbPick)
      - Right: Total time grouped by (Instance, nbPick), log-scale, with epoch line
    Only uses data with Ride_W2 == 0.

    Returns
    -------
    str
        Path to the saved figure.
    """
    # --- Read & filter data ---
    df = read_csv_with_encoding(data_path)
    df = df[df["Ride_W2"] == 0]

    # --- Common settings ---
    group_columns = ["Instance", "nbPick"]
    # categories mapping: nbPick -> label
    categories = {1: "1 Pickup", 2: "2 Pickups", 3: "3 Pickups", 4: "4 Pickups"}
    category_keys = list(categories.keys())
    category_labels = list(categories.values())

    # Color palette (same style as before)
    palette = sns.color_palette("gist_earth", n_colors=len(category_keys))
    palette = palette[::-1]  # reverse, like your previous functions
    alpha = 0.7
    palette_with_alpha = [(*c[:3], alpha) for c in palette]

    # --- Create figure & axes ---
    fig, (ax_time, ax_obj) = plt.subplots(1, 2, figsize=config.fig_size)

    # ---------- LEFT: Objective barplot ----------
    pivot_obj = prepare_grouped_data(
        df,
        group_columns=group_columns,
        agg_column="AvgObjective",
        agg_func="mean",
    )

    _plot_grouped_bars(
        ax=ax_obj,
        pivot_df=pivot_obj,
        categories=categories,          # dict is fine; labels come from values
        palette=palette_with_alpha,
        config=config,
        width=0.1,
        cluster_spacing=2.5,
        show_ylabel=True,
        ylabel="Avg. Objective / request (s)",
        xlabel="Instances",
        rotation=15,
        log_scale=False,
        ylim=None,
        target_line=None,
        grid=False,
    )

    # ---------- RIGHT: Time barplot ----------
    pivot_time = prepare_grouped_data(
        df,
        group_columns=group_columns,
        agg_column="Total time",
        agg_func="mean",
    )

    _plot_grouped_bars(
        ax=ax_time,
        pivot_df=pivot_time,
        categories=categories,
        palette=palette_with_alpha,
        config=config,
        width=0.1,
        cluster_spacing=2.5,
        show_ylabel=True,
        ylabel="Epoch Runtime (s)",
        xlabel="Instances",
        rotation=15,
        log_scale=True,            # log scale for runtime
        ylim=None,
        target_line=30,            # epoch size line
        grid=False,
    )

    # ---------- Legends ----------
    # Category legend
    darkened_palette = [darken_color(color, factor=0.5) for color in palette]
    legend_handles = [
        plt.Rectangle((0, 0), 1, 1,
                      facecolor=palette_with_alpha[i],
                      edgecolor=darkened_palette[i])
        for i in range(len(category_keys))
    ]

    fig.legend(
        handles=legend_handles,
        labels=category_labels,
        loc="upper center",
        bbox_to_anchor=(0.55, 0.99),
        ncol=4,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Pickup Limits",
        title_fontproperties={"weight": "bold", "size": config.legend_title_fsize},
        framealpha=1.0,
        facecolor="white",
    )

    # Target line legend (only relevant for the right subplot)
    fig.legend(
        handles=[mlines.Line2D([], [], color="red", linestyle="--", lw=1.2)],
        labels=["Epoch size (30s)"],
        loc="upper left",
        bbox_to_anchor=(0.2, 0.9),
        ncol=1,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor="none",
    )

    # --- Layout & save ---
    fig.tight_layout(rect=[0, 0, 1, 0.85])
    figure_path = os.path.join(
        os.path.dirname(data_path),
        "nbPick_obj_time_barplot.pdf"
    )
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path

def create_dropPick_wait_barplot_double(data_path: str, config: PlotConfig):
    # --- Create the double relative bar figure ---
    return create_double_relative_barplot(
        data_path=data_path,
        config=config,
        group_column="Instance",
        category_column="isDropPickPossible",
        value_column="AvgObjective",
        baseline_value=False,  # Drop-Pick not allowed is baseline
        comparison_value=True,  # Drop-Pick allowed is comparison
        ylabel="Relative Difference(%)",
        output_filename='dropPick_wait_barplot.pdf',
        ride_w2_values=(0.0, 0.5),
        color="lightsteelblue",
        width=0.5,
        rotation=15,
        # original code: ax.set_ylim(top=3.8) on both
        ylim_left=3.8,
        ylim_right=3.8,
        legend_label="Relative Difference(%)",
        major_locator=1,
        grid=False,
    )

# ---------------------------------------------------------------------------
# line / scatter plots
# ---------------------------------------------------------------------------
def create_pruning_scatter_plot_double(data_path: str, config: PlotConfig):
    plot_pruning_scatter_double(
        data_path=data_path,
        config=config,
        category_column="isSuccessorsLimited",
        group_column="Instance",
        metrics=("#LGenerated", "#LDominated"),
        categories=("noPruning", "pruneNodes", "pruneArcs", "discardSuboptimalPath"),
        category_labels=['No pruning', 'Prune nodes', 'Prune arcs', 'Prune suboptimal path'],
        output_filename='pruning_labels_comparison.pdf',
        ylabel='Number of Labels',
        ride_w2_values=(0.0, 0.5),
        log_scale=True,
        rotation=15,
        color_reverse=True,
        ylim=(2e6, 2e9)
    )

def create_pruning_scatter_plot_single(data_path: str, config: PlotConfig):
    plot_pruning_scatter_single(
        data_path=data_path,
        config=config,
        category_column="isSuccessorsLimited",
        group_column="Instance",
        metrics=("#LGenerated", "#LDominated"),
        categories=("noPruning", "pruneNodes", "pruneArcs", "discardSuboptimalPath"),
        category_labels=['No pruning', 'Prune nodes', 'Prune arcs', 'Prune suboptimal path'],
        output_filename='pruning_labels_scatter.pdf',
        ylabel='Number of labels',
        ride_w2_values=(0.0, 0.5),
        log_scale=True,
        rotation=15,
        color_reverse=True,
        ylim=(2e6, 2e9),
    )

def create_multiObj_wait_scatter(data_path: str, config: PlotConfig) -> str:
    # Marker shapes: circle, square, diamond
    marker_styles = ["o", "s", "D"]

    subplot_configs = []
    for i, inst_cat in enumerate(c.vehicle_groups):
        subplot_configs.append(
            {
                "item_column": "Instance",
                "category_column": "object_category",
                "value_column": "wait/req",
                "categories": c.OBJECTIVES,
                "category_labels": c.OBJECTIVES_labels,
                "ylabel": "Average waiting time (s)" if i == 0 else "",
                "xlabel": c.vehicle_groups_labels[i],
                "filter": lambda df, cat=inst_cat: df[df["Instance_category"] == cat],
                "show_legend": False,  # shared legend instead
                "show_grid": False,
                "rotation": 20,
                "line_width": 2,
                "marker_size": 5.2,
                "marker_styles": marker_styles,
                "ylim": (100, 480),
                "show_ylabel": i == 0,  # only left subplot has y-label
                "show_y_ticks": i == 0,  # hide y ticks on others
            }
        )

    # Shared legend configuration (will be placed once for all subplots)
    shared_legend_config = {
        "loc": "upper center",
        "bbox": (0.5, 1.08),  # slightly above the subplots
        "ncol": 3,
        "title": "Objective setting",
        "facecolor": "white",
    }

    figure_path = create_multi_subplot_lineplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs,
        output_filename="multiObj_wait_lineplots.pdf",
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=len(c.vehicle_groups),
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=None,
        figure_title=None,
        tight_layout_rect=None,
        additional_filter=None,
        color_reverse=True,
        palette_name="gist_earth",
        wspace=0.15,  # reduce gap between subplots
    )

    return figure_path


def create_multiObj_wait_grouped(data_path: str, config: PlotConfig) -> str:
    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="wait/req",
        category_column="object_category",
        categories=c.OBJECTIVES,
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="multiObj_wait_grouped.pdf",
        ylabel="Average waiting time (s)",
        category_labels=c.OBJECTIVES_labels,
        group_labels=c.vehicle_groups_labels,
        ylim=(100, 520),
        legend_title="Objective setting",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
    )


def create_multiObj_tripdelay_grouped(data_path: str, config: PlotConfig) -> str:
    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="tripDelay/req",
        category_column="object_category",
        categories=c.OBJECTIVES,
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="multiObj_tripdelay_grouped.pdf",
        ylabel="Average trip delay (s)",
        category_labels=c.OBJECTIVES_labels,
        group_labels=c.vehicle_groups_labels,
        ylim=(0, 220),
        legend_title="Objective setting",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
    )


def create_multiObj_passenger_grouped(data_path: str, config: PlotConfig) -> str:
    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="#pass in vehicle",
        category_column="object_category",
        categories=c.OBJECTIVES,
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="multiObj_passenger_grouped.pdf",
        ylabel="Average # passengers in vehicles",
        category_labels=c.OBJECTIVES_labels,
        group_labels=c.vehicle_groups_labels,
        ylim=(0.8, 2.9),
        legend_title="Objective setting",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
    )


def create_multiObj_idle_grouped(data_path: str, config: PlotConfig) -> str:
    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="idle time/vehicle",
        category_column="object_category",
        categories=c.OBJECTIVES,
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="multiObj_idle_grouped.pdf",
        ylabel="Average idle time/vehicle (s)",
        category_labels=c.OBJECTIVES_labels,
        group_labels=c.vehicle_groups_labels,
        ylim=(0.8, 1650),
        legend_title="Objective setting",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
        legend_loc="upper right",
    )


def create_custW3_wait_grouped(data_path: str, config: PlotConfig) -> str:
    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]

    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="wait/req",
        category_column="Req_W3",
        categories=[0, 1],
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="custW3_wait_grouped.pdf",
        ylabel="waiting time per request(s)",
        category_labels=category_labels,
        group_labels=c.vehicle_groups_labels,
        legend_title="Customer Weight",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
        ylim=(102, 360),
    )


def create_custW3_waitcust_grouped(data_path: str, config: PlotConfig) -> str:
    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]

    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="wait/cust",
        category_column="Req_W3",
        categories=[0, 1],
        group_column="Instance_category",
        groups=c.vehicle_groups,
        output_filename="custW3_waitcust_grouped.pdf",
        ylabel="waiting time per customer(s)",
        category_labels=category_labels,
        group_labels=c.vehicle_groups_labels,
        legend_title="Customer Weight",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
        ylim=(102, 360),
    )

def create_compare_wait_grouped(data_path: str, config: PlotConfig) -> str:
    return create_grouped_lineplot(
        data_path=data_path,
        config=config,
        value_column="wait/req",
        category_column="object_category",
        categories=c.OBJECTIVES[0:2],
        group_column="customer Group",
        groups=c.customer_groups,
        output_filename="compare_wait_grouped.pdf",
        ylabel="Average waiting time (s)",
        category_labels=c.OBJECTIVES_labels[0:2],
        group_labels=c.customer_groups,
        ylim=(82, 210),
        legend_title="Objective setting",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
    )

# ---------------------------------------------------------------------------
# profile plots
# ---------------------------------------------------------------------------

def create_multiObj_time_request_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'request_runtime_profiles')
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', 'Epoch'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=3)

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'EpochRuntime',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "Epoch time (s)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                #              "target_lines": 30,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbRequests',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "# Pending requests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_runtime_requests.pdf"
        )

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=(3,4),  # or config.fig_size_half
            sharex=True,  # share x-axis between subplots
            sharey=False,
            x_label_on_bottom_only=True,
            figure_title=None,
            shared_legend=False,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )


def create_multiObj_passenger_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'request_passengers_profiles')
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', 'Epoch'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=3)

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": '#passPerVehicle',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "# Passengers",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 15,
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                #              "target_lines": 30,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": '#requestPerVehicle',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "# Requests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 15,
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_request_passengers.pdf"
        )

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=config.fig_size,  # or config.fig_size_half
            sharex=True,  # share x-axis between subplots
            sharey=False,
            x_label_on_bottom_only=True,
            figure_title=None,
            shared_legend=True,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )

def create_multiObj_wait_delay_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'wait_delay_profiles')
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'ReadyTime', 'CommitWaitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )
    processed_df = processed_df[processed_df['TripDelay'] >= 0]
    processed_df['ElapsedTime'] = (processed_df['ReadyTime'] - 39600) / 60
    processed_df['WaitTime'] = processed_df['WaitTime'] / 60
    processed_df['TripDelay'] = processed_df['TripDelay'] / 60
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=3)
        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'WaitTime',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "Wait time (min)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 120),
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                #              "target_lines": 30,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'CommitWaitTime',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "Responce time (min)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 120),
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_wait_delay.pdf"
        )

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=config.fig_size,  # or config.fig_size_half
            sharex=True,  # share x-axis between subplots
            sharey=False,
            x_label_on_bottom_only=True,
            figure_title=None,
            shared_legend=True,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )
# ---------------------------------------------------------------------------
# Heat map
# ---------------------------------------------------------------------------

def create_multiObj_vehicle_heatmap(data_path: str, config: PlotConfig) -> None:

    # Read summary config CSV
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df['Instance_category'] == '4.High Demand']
    base_path = os.path.dirname(data_path)

    # Output folder
    main_folder_path = os.path.join(base_path, f'vehicle_epoch_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    folder_path = os.path.join(main_folder_path, 'vehicle_heatmaps')
    os.makedirs(folder_path, exist_ok=True)

    def build_file_path(row):
        mode = f"{c.MODE_DICT.get(row['Mode'])}_{row['Algorithm']}"
        file_name = c.DATA_TYPE_FILES['epochVehicle'].format(mode=mode)
        return os.path.join(base_path, row['Test_Folder'], file_name)

    # Loop over test rows
    for _, row in main_df.iterrows():
        file_path = build_file_path(row)
        results_df = read_csv_with_encoding(file_path)

        output_path = os.path.join(
            folder_path,
            f"{row['Instance']}_{row['Ride_W2']}_heatmap.pdf"
        )

        create_single_heatmap_figure(
            df=results_df,
            config=config,
            row_column="VehicleID",
            col_column="Epoch",
            value_column="nbRequests",
            output_path=output_path,
            xlabel="Runtime Epochs",
            ylabel="Vehicles",
            cbar_label="# Requests planned per route",
  #          max_rows=row['#vehicles'],  # 1 per (N/100) vehicles
            max_rows=200,  # 1 per (N/100) vehicles
            vmin = 0,
            vmax = 4,
            cmap='YlGnBu',
        )

        print(f"Saved vehicle heatmap → {output_path}")


def create_multiObj_vehicle_std_plot(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)

    main_folder_path = os.path.join(base_path, f'vehicle_epoch_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    metrics = ['nbRequests', 'nbNodes', 'length', 'nbOnboards_nodes', 'nbCommitted', 'avgPassPerStop']
    metrics_label = ['#Requests', '#Stops', 'length', '#Onboards', '#Commits' , 'Pass/Stop']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='epochVehicle',
        value_column=metrics,
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=['std', 'mean'],
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_names = processed_df['Instance'].unique()

    palette = sns.color_palette("gist_earth", n_colors=3)

    # Loop over each test instance
    # Loop through all metrics
    for metric, label in zip(metrics, metrics_label):
        # Folder for all plots
        folder_path = os.path.join(main_folder_path, f'vehicle_epoch_{metric}_plots')
        os.makedirs(folder_path, exist_ok=True)

        for test in test_names:
            result_df = processed_df[processed_df['Instance'] == test]

            # Two-row subplot config: std + mean
            subplot_cfgs = [
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'std_{metric}',
                    "group_column": 'object_category',
                    "groups": c.OBJECTIVES,
                    "group_labels": c.OBJECTIVES_labels,
                    "ylabel": f"{label} (std)",
                    "xlabel": "Elapsed Time (min)",
                    "show_xlabel": False,
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 15,
                    "grid": True,
                    "show_legend": True,
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                },
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'mean_{metric}',
                    "group_column": 'object_category',
                    "groups": c.OBJECTIVES,
                    "group_labels": c.OBJECTIVES_labels,
                    "ylabel": f"{label} (mean)",
                    "xlabel": "Elapsed Time (min)",
                    "show_xlabel": True,
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 15,
                    "grid": True,
                    "show_legend": True,
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                }
            ]

            # Save plot
            output_path = os.path.join(folder_path, f"{test}_{metric}.pdf")

            create_multi_subplot_profile_lineplots(
                df=result_df,
                config=config,
                subplot_configs=subplot_cfgs,
                output_path=output_path,
                fig_size=(3,4),
                sharex=True,
                sharey=False,
                x_label_on_bottom_only=True,
                figure_title=None,
                shared_legend=True,
                shared_legend_config={
                    "loc": "upper center",
                    "bbox": (0.5, 0.98),
                    "ncol": 3,
                    "title": "Object Settings",
                },
                tight_layout_rect=[0, 0, 1, 0.88],
            )


def create_multiObj_vehicle_std_shade_plot(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)

    main_folder_path = os.path.join(base_path, f'vehicle_epoch_shade_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    metrics = ['nbRequests', 'nbNodes', 'length', 'nbOnboards', 'nbCommitted']
    labels = ['# Requests', '# Nodes', 'length', '# Onboards', '# Committed']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='epochVehicle',
        value_column=metrics,
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=['std', 'mean'],
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60

    palette = sns.color_palette("gist_earth", n_colors=3)

    for metric, label in zip(metrics, labels):
        folder_path = os.path.join(main_folder_path, f'vehicle_shaded_{metric}_plots')
        os.makedirs(folder_path, exist_ok=True)

        for test in processed_df['Instance'].unique():
            df_test = processed_df[processed_df['Instance'] == test]


            create_shaded_mean_std_plot(
                df=df_test,
                metric=metric,
                label=label,
                test=test,
                config=config,
                palette=palette,
                output_folder=folder_path,
            )


def create_multiObj_time_dual_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'dual_route_profiles')
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['waitTime', 'EpochRuntime', 'TripDelay', 'nbNewRequests', 'Epoch', 'nbRequests'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=3)

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbNewRequests',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "nbNewRequests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 15,
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                #              "target_lines": 30,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbRequests',
                "group_column": 'object_category',
                "groups": c.OBJECTIVES,
                "group_labels": c.OBJECTIVES_labels,
                "ylabel": "nbRequests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 15,
                "grid": True,
                "show_legend": True,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_runtime_requests.pdf"
        )

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=config.fig_size,  # or config.fig_size_half
            sharex=True,  # share x-axis between subplots
            sharey=False,
            x_label_on_bottom_only=True,
            figure_title=None,
            shared_legend=True,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )



def create_compare_barplot_single(
    data_path: str,
    data_path_ISUD: str,
    config: PlotConfig
) -> str:
    import os
    import pandas as pd
    from matplotlib import pyplot as plt
    import numpy as np

    """
    Create a single figure:
      - Average waiting time (wait/req, converted to minutes) grouped by
        (customer Group, Algorithm_Name + W_delay),
        using data from two datasets combined, plus manual M-RTRS rows.

    Parameters
    ----------
    data_path : str
        Path to the first CSV file.
    data_path_ISUD : str
        Path to the second CSV file (ISUD).
    config : PlotConfig
        Plot configuration object.

    Returns
    -------
    str
        Path to the saved figure.
    """

    # -----------------------------
    # 1. Read data from both sources
    # -----------------------------
    df_main = read_csv_with_encoding(data_path)
    df_isud = read_csv_with_encoding(data_path_ISUD)

    # -----------------------------
    # 2. Convert wait/req to minutes
    #    (assuming original unit is seconds)
    # -----------------------------
    if "wait/req" in df_main.columns:
        df_main["wait/req"] = df_main["wait/req"] / 60.0
    if "wait/req" in df_isud.columns:
        df_isud["wait/req"] = df_isud["wait/req"] / 60.0

    # -----------------------------
    # 3. Helper: encode W_delay into Algorithm_Name
    # -----------------------------
    def _add_alg_with_delay(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()

        def _alg_with_delay(row):
            if row["Ride_W2"] == 0:
                suffix = r" $(\omega_{drop}$ = 0)"
            elif row["Ride_W2"] == 0.5:
                suffix = r" $(\omega_{drop}$ = 0.5)"
            else:
                # fallback for any unexpected value
                suffix = rf" $(W_{{delay}}$ = {row['Ride_W2']})"
            return f"{row['Algorithm_Name']}{suffix}"

        df["Algorithm_Name"] = df.apply(_alg_with_delay, axis=1)
        return df

    df_main = _add_alg_with_delay(df_main)
    df_isud = _add_alg_with_delay(df_isud)


    # -----------------------------
    # 5. Create extra rows for M-RTRS
    #    (values are already in MINUTES)
    # -----------------------------
    extra_rows = pd.DataFrame({
        "customer Group": c.customer_groups,
        "Algorithm_Name": ["M-RTRS"] * 3,
        "Ride_W2": [0, 0, 0],
        "wait/req": [2.33, 3.83, 3.78],
    })
    extra_rows = _add_alg_with_delay(extra_rows)

    # -----------------------------
    # 6. Combine all datasets
    # -----------------------------
    df = pd.concat([df_main, df_isud, extra_rows], ignore_index=True)

    # Ensure fixed order for customer Group
    df["customer Group"] = pd.Categorical(
        df["customer Group"],
        categories=c.customer_groups,
        ordered=True,
    )

    # -----------------------------
    # 7. Plot settings: group by
    # -----------------------------
    group_columns = ["customer Group", "Algorithm_Name"]


    preferred_algo_order = [
        r"M-RTRS $(\omega_{drop}$ = 0)",
        r"F-ICG $(\omega_{drop}$ = 0)",
        r"B-CG-C $(\omega_{drop}$ = 0)",
        r"B-CG-C $(\omega_{drop}$ = 0.5)",
    ]

    unique_algos = list(df["Algorithm_Name"].unique())

    # Build category_keys honoring preferred order first, then any others
    category_keys = (
            [a for a in preferred_algo_order if a in unique_algos] +
            [a for a in unique_algos if a not in preferred_algo_order]
    )

    categories = {k: k for k in category_keys}
    category_labels = list(categories.values())

    # Color palette
    palette = sns.color_palette("gist_earth", n_colors=len(category_keys))[::-1]
    alpha = 0.7
    palette_with_alpha = [(*color[:3], alpha) for color in palette]

    # -----------------------------
    # 8. Create figure & axis
    # -----------------------------
    fig, ax_obj = plt.subplots(1, 1, figsize=(4.5, 4))

    # ---------- Waiting-time barplot (combined datasets) ----------
    pivot_obj = prepare_grouped_data(
        df,
        group_columns=group_columns,
        agg_column="wait/req",
        agg_func="mean",
    )

    # Try to enforce customer group order on the pivot index if applicable
    try:
        pivot_obj = pivot_obj.reindex(c.customer_groups)
    except Exception:
        # If pivot_obj doesn't use customer Group as a simple index, just skip
        pass

    _plot_grouped_bars(
        ax=ax_obj,
        pivot_df=pivot_obj,
        categories=categories,
        palette=palette_with_alpha,
        config=config,
        width=0.1,
        cluster_spacing=2.5,
        show_ylabel=True,
        ylabel="Average Waiting Time (min)",
        xlabel="Customer Group",
        rotation=0,
        log_scale=False,
        ylim=(0,5.5),
        target_line=None,
        grid=False,
    )

    # -----------------------------
    # 9. Add data labels INSIDE bars
    # -----------------------------
    ymin, ymax = ax_obj.get_ylim()
    # small offset downward so text sits inside the bar
    y_offset = (ymax - ymin) * 0.02

    label_fontsize = max(6, getattr(config, "tick_fsize", 9) - 2)

    for patch in ax_obj.patches:
        height = patch.get_height()
        if np.isnan(height) or height <= 0:
            continue

        x = patch.get_x() + patch.get_width() / 2.0
        y = height - y_offset  # inside the bar

        # Color of the bar
        bar_color = patch.get_facecolor()
        text_color = (
            max(bar_color[0] - 0.4, 0),
            max(bar_color[1] - 0.4, 0),
            max(bar_color[2] - 0.4, 0),
            1.0
        )

        ax_obj.text(
            x,
            y,
            f"{height:.2f}",
            ha="center",
            va="top",
            color=text_color,
            rotation=90,  # rotate label
            fontsize=label_fontsize,  # smaller
            fontweight="bold",
        )

    # -----------------------------
    # 9. Legend
    # -----------------------------
    darkened_palette = [darken_color(color, factor=0.5) for color in palette]
    legend_handles = [
        plt.Rectangle((0, 0), 1, 1,
                      facecolor=palette_with_alpha[i],
                      edgecolor=darkened_palette[i])
        for i in range(len(category_keys))
    ]

    fig.legend(
        handles=legend_handles,
        labels=category_labels,
        loc="upper left",
        bbox_to_anchor=(0.15, 0.99),
        ncol=1,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Algorithms",
        title_fontproperties={"weight": "bold", "size": config.legend_title_fsize},
        framealpha=1.0,
        facecolor="white",
    )

    # -----------------------------
    # 10. Layout & save
    # -----------------------------
    fig.tight_layout()
    figure_path = os.path.join(
        os.path.dirname(data_path),
        "compare_wait_time_barplot.pdf"
    )
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path

def create_compare_isud_barplot_single(
    data_path: str,
    data_path_ISUD: str,
    config: PlotConfig
) -> str:
    import os
    import pandas as pd
    from matplotlib import pyplot as plt
    import numpy as np

    """
    Create a single figure:
      - Average waiting time (wait/req, converted to minutes) grouped by
        (customer Group, Algorithm_Name + W_delay),
        using data from two datasets combined, plus manual M-RTRS rows.

    Parameters
    ----------
    data_path : str
        Path to the first CSV file.
    data_path_ISUD : str
        Path to the second CSV file (ISUD).
    config : PlotConfig
        Plot configuration object.

    Returns
    -------
    str
        Path to the saved figure.
    """

    # -----------------------------
    # 1. Read data from both sources
    # -----------------------------
    df_isud = read_csv_with_encoding(data_path_ISUD)

    # -----------------------------
    # 2. Convert wait/req to minutes
    #    (assuming original unit is seconds)
    # -----------------------------
    if "wait/req" in df_isud.columns:
        df_isud["wait/req"] = df_isud["wait/req"] / 60.0

    # -----------------------------
    # 3. Helper: encode W_delay into Algorithm_Name
    # -----------------------------
    def _add_alg_with_delay(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()

        def _alg_with_delay(row):
            if row["Ride_W2"] == 0:
                suffix = r" $(\omega_{drop}$ = 0)"
            elif row["Ride_W2"] == 0.5:
                suffix = r" $(\omega_{drop}$ = 0.5)"
            else:
                # fallback for any unexpected value
                suffix = rf" $(W_{{delay}}$ = {row['Ride_W2']})"
            return f"{row['Algorithm_Name']}"

        df["Algorithm_Name"] = df.apply(_alg_with_delay, axis=1)
        return df

    df_isud = _add_alg_with_delay(df_isud)


    # -----------------------------
    # 5. Create extra rows for M-RTRS
    #    (values are already in MINUTES)
    # -----------------------------
    extra_rows = pd.DataFrame({
        "customer Group": c.customer_groups,
        "Algorithm_Name": ["M-RTRS"] * 3,
        "Ride_W2": [0, 0, 0],
        "wait/req": [2.33, 3.83, 3.78],
    })
    extra_rows = _add_alg_with_delay(extra_rows)

    # -----------------------------
    # 6. Combine all datasets
    # -----------------------------
    df = pd.concat([df_isud, extra_rows], ignore_index=True)

    # Ensure fixed order for customer Group
    df["customer Group"] = pd.Categorical(
        df["customer Group"],
        categories=c.customer_groups,
        ordered=True,
    )

    # -----------------------------
    # 7. Plot settings: group by
    # -----------------------------
    group_columns = ["customer Group", "Algorithm_Name"]


    preferred_algo_order = [
        r"M-RTRS",
        r"F-ICG",
    ]

    unique_algos = list(df["Algorithm_Name"].unique())

    # Build category_keys honoring preferred order first, then any others
    category_keys = (
            [a for a in preferred_algo_order if a in unique_algos] +
            [a for a in unique_algos if a not in preferred_algo_order]
    )

    categories = {k: k for k in category_keys}
    category_labels = list(categories.values())



    # Color palette
    palette = sns.color_palette("gist_earth", n_colors=4)[::-1]
    alpha = 0.7
    palette_with_alpha = [(*color[:3], alpha) for color in palette]

    # -----------------------------
    # 8. Create figure & axis
    # -----------------------------
    fig, ax_obj = plt.subplots(1, 1, figsize=(3.5, 4))

    # ---------- Waiting-time barplot (combined datasets) ----------
    pivot_obj = prepare_grouped_data(
        df,
        group_columns=group_columns,
        agg_column="wait/req",
        agg_func="mean",
    )

    # Try to enforce customer group order on the pivot index if applicable
    try:
        pivot_obj = pivot_obj.reindex(c.customer_groups)
    except Exception:
        # If pivot_obj doesn't use customer Group as a simple index, just skip
        pass

    _plot_grouped_bars(
        ax=ax_obj,
        pivot_df=pivot_obj,
        categories=categories,
        palette=palette_with_alpha,
        config=config,
        width=0.2,
        cluster_spacing=1.8,
        show_ylabel=True,
        ylabel="Average Waiting Time (min)",
        xlabel="Customer Group",
        rotation=0,
        log_scale=False,
        ylim=(0,5.5),
        target_line=None,
        grid=False,
    )

    # -----------------------------
    # 9. Add data labels INSIDE bars
    # -----------------------------
    ymin, ymax = ax_obj.get_ylim()
    # small offset downward so text sits inside the bar
    y_offset = (ymax - ymin) * 0.02

    label_fontsize = max(6, getattr(config, "tick_fsize", 9) - 2)

    for patch in ax_obj.patches:
        height = patch.get_height()
        if np.isnan(height) or height <= 0:
            continue

        x = patch.get_x() + patch.get_width() / 2.0
        y = height - y_offset  # inside the bar

        # Color of the bar
        bar_color = patch.get_facecolor()
        text_color = (
            max(bar_color[0] - 0.4, 0),
            max(bar_color[1] - 0.4, 0),
            max(bar_color[2] - 0.4, 0),
            1.0
        )

        ax_obj.text(
            x,
            y,
            f"{height:.2f}",
            ha="center",
            va="top",
            color=text_color,
            rotation=90,  # rotate label
            fontsize=label_fontsize,  # smaller
            fontweight="bold",
        )

    # -----------------------------
    # 9. Legend
    # -----------------------------
    darkened_palette = [darken_color(color, factor=0.5) for color in palette]
    legend_handles = [
        plt.Rectangle((0, 0), 1, 1,
                      facecolor=palette_with_alpha[i],
                      edgecolor=darkened_palette[i])
        for i in range(len(category_keys))
    ]

    fig.legend(
        handles=legend_handles,
        labels=category_labels,
        loc="upper left",
        bbox_to_anchor=(0.15, 0.99),
        ncol=1,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Algorithms",
        title_fontproperties={"weight": "bold", "size": config.legend_title_fsize},
        framealpha=1.0,
        facecolor="white",
    )

    # -----------------------------
    # 10. Layout & save
    # -----------------------------
    fig.tight_layout()
    figure_path = os.path.join(
        os.path.dirname(data_path),
        "compare_wait_time_isud_barplot.pdf"
    )
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path

def create_multiObj_gap_violinplots(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    main_df['GAP'] = 100 * (main_df['Objective'] - main_df['LinearObjective']) / main_df['Objective']

    # Preprocess data (same as before)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['GAP', 'EpochRuntime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'GAP']
    )

    # Save to temporary file
    temp_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')
    processed_df.to_csv(temp_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category',
            'value_column': 'GAP',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Integrality Gap (%)',
            'xlabel': 'Integrality Gap',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
    ]



    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_violinplots(
        data_path=temp_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f"multiObj_gap_violinplot_{outlier}.pdf",
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=1,
        shared_legend=True,  # for category colors
        shared_legend_config=shared_legend_config,
        shared_stats_legend=True,  # to show Mean/Median/5–95%/Distribution legend
        shared_stats_legend_config={
            "loc": "upper right",
            "bbox": (0.98, 0.98),
            "ncol": 1,
        },
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path


def create_ablation_iter_violinplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
   # main_df = main_df[main_df['Dynamic_Pricing'] == True]
    # Preprocess commit wait time data

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    processed_df_0 = processed_df[processed_df['Ride_W2'] == 0]
    processed_df_5 = processed_df[processed_df['Ride_W2'] == 0.5]

    processed_df_0 = processed_df_0[processed_df_0['Instance_category'] != '4.High Demand']
    processed_df_5 = processed_df_5[processed_df_5['Instance_category'] != '4.High Demand']

    temp_path_0 = os.path.join(os.path.dirname(data_path), 'time_temp_0.csv')
    processed_df_0.to_csv(temp_path_0, index=False)

    temp_path_5 = os.path.join(os.path.dirname(data_path), 'time_temp_5.csv')
    processed_df_5.to_csv(temp_path_5, index=False)

    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'data_df': processed_df_0,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': '#SP Iter',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': '# Iterations per Epoch',
            'xlabel': r"$\omega_{drop}$ = 0",
            'rotation': 15,
            'width': 0.38,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': [0.7, 7.2],
        },
        {  # Right: Problem Size
            'data_df': processed_df_5,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': '#SP Iter',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': '# Iterations per Epoch',
            'xlabel': r"$\omega_{drop}$ = 0.5",
            'rotation': 15,
            'width': 0.38,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim' : [0.7, 7.2],
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.3, 0.99),
        'ncol': 2,
        'title': 'Setting'
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f"ablation_iter_violinplot_{outlier}.pdf",
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,  # for category colors
        shared_legend_config=shared_legend_config,
        shared_stats_legend=True,  # to show Mean/Median/5–95%/Distribution legend
        shared_stats_legend_config={
            "loc": "upper right",
            "bbox": (0.94, 0.95),
            "ncol": 2,
        },
        tight_layout_rect=(0, 0, 1, 0.85)
    )

    # Clean up
 #   if os.path.exists(temp_path):
 #       os.remove(temp_path)

    return figure_path

def create_ablation_iter_violinplot_single(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
   # main_df = main_df[main_df['Dynamic_Pricing'] == True]
    # Preprocess commit wait time data

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    processed_df_0 = processed_df[processed_df['Ride_W2'] == 0]
    processed_df_5 = processed_df[processed_df['Ride_W2'] == 0.5]

    processed_df_0 = processed_df_0[processed_df_0['Instance_category'] != '4.High Demand']
    processed_df_5 = processed_df_5[processed_df_5['Instance_category'] != '4.High Demand']

    temp_path_0 = os.path.join(os.path.dirname(data_path), 'time_temp_0.csv')
    processed_df_0.to_csv(temp_path_0, index=False)

    temp_path_5 = os.path.join(os.path.dirname(data_path), 'time_temp_5.csv')
    processed_df_5.to_csv(temp_path_5, index=False)

    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'data_df': processed_df_0,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels[0:3],
            'category_column': 'setting',
            'value_column': '#SP Iter',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': '# Iterations per Epoch',
            'xlabel': "CG Iterations ",
            'rotation': 15,
            'width': 0.38,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': [0.7, 7.2],
        },
    ]

    subplot_configs_2 = [
        {  # Right: Problem Size
            'data_df': processed_df_5,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels[0:3],
            'category_column': 'setting',
            'value_column': '#SP Iter',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': '# Iterations per Epoch',
            'xlabel': "CG Iterations ",
            'rotation': 15,
            'width': 0.38,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': [0.7, 7.2],
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.3, 0.99),
        'ncol': 2,
        'title': 'Setting'
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f"ablation_iter_violinplot_0_{outlier}.pdf",
        fig_size=(3, 4),
        n_rows=1,
        n_cols=1,
        shared_legend=False,  # for category colors
 #       shared_legend_config=shared_legend_config,
        shared_stats_legend=True,  # to show Mean/Median/5–95%/Distribution legend
        shared_stats_legend_config={
            "loc": "upper right",
            "bbox": (0.9, 0.99),
            "ncol": 3,
        },
        tight_layout_rect=(0, 0, 1, 0.9)
    )

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f"ablation_iter_violinplot_5_{outlier}.pdf",
        fig_size=(3, 4),
        n_rows=1,
        n_cols=1,
        shared_legend=False,  # for category colors
  #      shared_legend_config=shared_legend_config,
        shared_stats_legend=True,  # to show Mean/Median/5–95%/Distribution legend
        shared_stats_legend_config={
            "loc": "upper right",
            "bbox": (0.9, 0.99),
            "ncol": 3,
        },
        tight_layout_rect=(0, 0, 1, 0.9)
    )

    # Clean up
 #   if os.path.exists(temp_path):
 #       os.remove(temp_path)

    return figure_path

def create_dynamic_iter_violinplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'Dynamic_Pricing']
    )

    processed_df_0 = processed_df[processed_df['Ride_W2'] == 0]
    processed_df_5 = processed_df[processed_df['Ride_W2'] == 0.5]

    temp_path_0 = os.path.join(os.path.dirname(data_path), 'time_temp_0.csv')
    processed_df_0.to_csv(temp_path_0, index=False)

    temp_path_5 = os.path.join(os.path.dirname(data_path), 'time_temp_5.csv')
    processed_df_5.to_csv(temp_path_5, index=False)

    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'data_df': processed_df_0,
            'item_column': 'Instance',
            'category_column': 'Dynamic_Pricing',
            'value_column': '#SP Iter',
            'categories': ['False', 'True'],
            'category_labels': ['2 Pickups', 'Dynamic Pickups Limit'],
            'ylabel': '# Iterations per Epoch',
            'xlabel': r"$\omega_{drop}$ = 0",
            'rotation': 15,
            'width': 0.65,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': [0.7, 5.7],
        },
        {  # Right: Problem Size
            'data_df': processed_df_5,
            'item_column': 'Instance',
            'category_column': 'Dynamic_Pricing',
            'value_column': '#SP Iter',
            'categories': ['False', 'True'],
            'category_labels': ['2 Pickups', 'Dynamic Pickups Limit'],
            'ylabel': '# Iterations per Epoch',
            'xlabel': r"$\omega_{drop}$ = 0.5",
            'rotation': 15,
            'width': 0.65,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim' : [0.7, 5.7],
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.3, 0.99),
        'ncol': 2,
        'title': 'Pickup Limits'
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f"dynamic_iter_violinplot_{outlier}.pdf",
        fig_size=(10, 5) if config.fig_size is None else config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,  # for category colors
        shared_legend_config=shared_legend_config,
        shared_stats_legend=True,  # to show Mean/Median/5–95%/Distribution legend
        shared_stats_legend_config={
            "loc": "upper right",
            "bbox": (0.94, 0.95),
            "ncol": 2,
        },
        tight_layout_rect=(0, 0, 1, 0.87)
    )

    # Clean up
 #   if os.path.exists(temp_path):
 #       os.remove(temp_path)

    return figure_path

def create_runtime_iter_box_violin(
    data_path: str,
    config: PlotConfig,
    output_filename: str = "dynamic_iter_box_violin.pdf",
    color_reverse: bool = False,
    palette_name: str = "gist_earth",
    palette=None,
) -> str:
    """
    Create one figure with two subplots, only for Ride_W2 == 0:

        LEFT  : boxplot of EpochRuntime
        RIGHT : half-violin plots of #SP Iter

    Categories: Dynamic_Pricing ∈ ['False', 'True']
                labeled as ['2 Pickups', 'Dynamic Pickups Limit']
    """
    import numpy as np
    from matplotlib import pyplot as plt
    import matplotlib.lines as mlines
    from Visualization.visualize_violin_plots import prepare_violin_data, plot_violin_group
    from Visualization.visualize_box_plots import prepare_boxplot_data, plot_boxplot_group
    # ------------------------------------------------------------------
    # 1. Read and prepare data
    # ------------------------------------------------------------------
    main_df = read_csv_with_encoding(data_path)

    # BOX side: just filter Ride_W2 == 0
    df_box = main_df[main_df["Ride_W2"] == 0].copy()

    # VIOLIN side: use same preprocessing as your existing violin function
    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type="time",
        value_column=["#SP Iter"],
        instance_column="Instance",
        algorithm_column="Algorithm",
        aggregate_func=None,  # keep all values
        additional_columns=["Ride_W2", "Dynamic_Pricing"],
    )
    df_violin = processed_df[processed_df["Ride_W2"] == 0].copy()

    # Make sure grouping columns are strings
    df_box["Instance"] = df_box["Instance"].astype(str)
    df_box["Dynamic_Pricing"] = df_box["Dynamic_Pricing"].astype(str)
    df_violin["Instance"] = df_violin["Instance"].astype(str)
    df_violin["Dynamic_Pricing"] = df_violin["Dynamic_Pricing"].astype(str)

    # Common x-axis items and categories

    x_items = np.sort(df_box["Instance"].unique()).astype(str)
    categories = ["False", "True"]
    category_labels = ["2 Pickups", "Dynamic Pickups Limit"]

    # ------------------------------------------------------------------
    # 2. Palette
    # ------------------------------------------------------------------
    if palette is None:
        palette = sns.color_palette(palette_name, n_colors=len(categories))
    if color_reverse:
        palette = palette[::-1]

    # ------------------------------------------------------------------
    # 3. Prepare data for BOX subplot (EpochRuntime)
    # ------------------------------------------------------------------
    box_plot_data = prepare_boxplot_data(
        df=df_box,
        items=x_items,
        items_column="Instance",
        categories=categories,
        category_column="Dynamic_Pricing",
        value_column="EpochRuntime",
    )

    # ------------------------------------------------------------------
    # 4. Prepare data for VIOLIN subplot (#SP Iter)
    # ------------------------------------------------------------------
    violin_plot_data, _ = prepare_violin_data(
        df=df_violin,
        items=x_items,
        items_column="Instance",
        categories=categories,
        category_column="Dynamic_Pricing",
        value_column="#SP Iter",
        compute_zero_pct=False,
    )

    # ------------------------------------------------------------------
    # 5. Figure / axes
    # ------------------------------------------------------------------
    fig_size = config.fig_size


    fig, (ax_box, ax_violin) = plt.subplots(1, 2, figsize=fig_size)

    # ------------------------------------------------------------------
    # 6. LEFT: EpochRuntime boxplots
    # ------------------------------------------------------------------
    box_handles, box_labels = plot_boxplot_group(
        ax=ax_box,
        plot_data=box_plot_data,
        categories=categories,
        palette=palette,
        config=config,
        show_outliers=outlier,
        show_ylabel=True,
        ylabel="Epoch Runtime (s)",
        width=0.55,
        gap_factors=None,
        target_lines=30,  # epoch size
        xlabel="Instances",
        x_tick_labels=x_items,
        rotation=15,
        ylim=None,
        show_legend=False,  # we'll build shared legends manually
        legend_loc="best",
        legend_bbox=None,
        legend_title=None,
        category_labels=category_labels,
        show_grid=False,
        ylabel_pad=None,
        legend_ncol=1,
    )

    # ------------------------------------------------------------------
    # 7. RIGHT: #SP Iter half-violins
    #     (plot_violin_group already does half-violins when len(categories)==2)
    # ------------------------------------------------------------------
    cat_handles_violin, cat_labels_violin, stats_handles, stats_labels = plot_violin_group(
        ax=ax_violin,
        plot_data=violin_plot_data,
        categories=categories,
        palette=palette,
        config=config,
        show_ylabel=True,
        ylabel="# Iterations per Epoch",
        width=0.65,
        gap_factors=None,
        target_lines=None,
        xlabel=r"$\omega_{drop} = 0$",
        x_tick_labels=x_items,
        rotation=15,
        ylim=[0.7, 5.7],
        show_category_legend=False,  # shared legend instead
        legend_loc="best",
        legend_bbox=None,
        legend_title=None,
        legend_ncol=1,
        category_labels=category_labels,
        show_grid=False,
        ylabel_pad=None,
        zero_pct_data=None,
        zero_y_offset=-0.012,
    )

    # ------------------------------------------------------------------
    # 8. Shared legends
    # ------------------------------------------------------------------
    # Category legend ("Pickup Limits") – use category handles
    # from either box or violin; they correspond to the same categories.
    cat_handles = cat_handles_violin  # nicer, uses violin patches
    cat_labels = category_labels

    fig.legend(
        handles=cat_handles,
        labels=cat_labels,
        loc="upper center",
        bbox_to_anchor=(0.34, 0.99),
        ncol=2,
        fontsize=config.legend_fsize,
        edgecolor=config.legend_edgecolor,
        title="Pickup Limits",
        title_fontproperties={
            "weight": "bold",
            "size": config.legend_title_fsize,
        },
        framealpha=1.0,
        facecolor="white",
    )

    # Stats legend (Median / Mean / 5–95% / Distribution) from violin
    fig.legend(
        handles=stats_handles,
        labels=stats_labels,
        loc="upper right",
        bbox_to_anchor=(0.95, 0.95),
        ncol=2,
        fontsize=config.legend_fsize,
        edgecolor=config.legend_edgecolor,
        framealpha=1.0,
        facecolor="white",
    )

    # Optional: separate legend for epoch-size target line (30s)
    fig.legend(
        handles=[mlines.Line2D([], [], color="red", linestyle="--", lw=1.5)],
        labels=[f"Epoch size (30s)"],
        loc="upper center",
        bbox_to_anchor=(0.225, 0.89),
        ncol=1,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor="none",
    )

    # ------------------------------------------------------------------
    # 9. Layout & save
    # ------------------------------------------------------------------
    fig.tight_layout(rect=(0, 0, 1, 0.87))

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path)
    plt.close(fig)

    return figure_path


def create_ablation_request_boxplot_triple(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)

    processed_request_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    processed_request_df_0 = processed_request_df[processed_request_df['Ride_W2'] == 0]
    processed_request_df_5 = processed_request_df[processed_request_df['Ride_W2'] == 0.5]

    processed_time_df_0 = processed_time_df[processed_time_df['Ride_W2'] == 0]
    processed_time_df_5 = processed_time_df[processed_time_df['Ride_W2'] == 0.5]

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df_0,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': 'WaitTime',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip delay
  #          'data_path': request_path,
            'data_df': processed_time_df_0,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': 'EpochRuntime',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Runtime',
            'rotation': 20,
            'target_lines': 30,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df_0,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': '#passPerVehicle',
            'categories': c.param_to_setting_labels,
            'category_labels': c.param_to_setting_labels,
            'ylabel': 'Initial Vehicle Load',
            'xlabel': 'Initial Onboard Passengers',
            'rotation': 20,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.6, 0.99),
        'ncol': 4,
        'title': 'Setting'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'ablation_triple_boxplot_{outlier}.pdf',
        fig_size=(9.5, 4),
        n_rows=1,
        n_cols=3,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
    )

    return figure_path


def create_ablation_request_boxplot_fourth(data_path: str, config: PlotConfig,
                                          param_map=None, param_labels=None) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    _param_map    = param_map    or c.param_to_setting
    _param_labels = param_labels or c.param_to_setting_labels
    main_df['setting'] = main_df['paramFile'].map(_param_map)

    processed_request_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='request',
        value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        filter_condition=lambda df: df[df['TripDelay'] >= 0],
        transform_func=lambda x: x / 60,  # seconds -> minutes
        aggregate_func=None,
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )
    processed_request_df['TripDelay'] = processed_request_df['TripDelay']-0.5

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Ride_W2', 'setting', 'Instance_category']
    )

    processed_request_df_0 = processed_request_df[processed_request_df['Ride_W2'] == 0]
    processed_request_df_5 = processed_request_df[processed_request_df['Ride_W2'] == 0.5]

    processed_time_df_0 = processed_time_df[processed_time_df['Ride_W2'] == 0]
    processed_time_df_5 = processed_time_df[processed_time_df['Ride_W2'] == 0.5]

    data_df_time = processed_time_df_0
    data_df_request = processed_request_df_0
    num = 0

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 2: Trip delay
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': 'EpochRuntime',
            'categories': _param_labels,
            'category_labels': _param_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
  #          'ylim': (0,68),
            'ylim': (0, 305),
            'rotation': 15,
            'target_lines': 30,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 3. nbRequests
            #      'data_path': time_path,  # <--- per-subplot
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': '#passPerVehicle',
            'categories': _param_labels,
            'category_labels': _param_labels,
            'ylabel': 'Initial Vehicle Load',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },

        {  # 3. nbRequests
            #      'data_path': time_path,  # <--- per-subplot
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': 'nbRequests',
            'categories': _param_labels,
            'category_labels': _param_labels,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            'rotation': 15,
 #           'ylim': (0, 850),
            'ylim': (0, 2950),
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'setting',
            'value_column': 'WaitTime',
            'categories': _param_labels,
            'category_labels': _param_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.4,
  #          'ylim': (0, 18.2),
            'ylim': (0, 34),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 4,
        'title': 'Setting'
    }
    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.33, 0.9)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'ablation_fourth_boxplot_{num}_{outlier}.pdf',
        fig_size=(13, 4),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

    return figure_path


def create_multiObj_multi_std_plot(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)

    main_folder_path = os.path.join(base_path, 'vehicle_epoch_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    # We will produce ONE figure per test, containing all metrics
    output_folder = os.path.join(main_folder_path, 'vehicle_epoch_all_metrics_plots')
    os.makedirs(output_folder, exist_ok=True)

    metrics = ['nbRequests', 'nbNodes', 'length']
    metrics_label = ['#Requests', '#Stops', 'length']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='epochVehicle',
        value_column=metrics,
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=['std', 'mean'],
        additional_columns=['object_category', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_names = processed_df['Instance'].unique()

    palette = sns.color_palette("gist_earth", n_colors=3)

    # ---- Loop over each test instance (ONE figure per test) ----
    for test in test_names:
        result_df = processed_df[processed_df['Instance'] == test]

        subplot_cfgs = []

        # We want a 2x3 grid: first row = std, second row = mean
        # columns: nbRequests | nbNodes | length
        for metric, label in zip(metrics, metrics_label):
            # Row 1: std
            subplot_cfgs.append(
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'std_{metric}',
                    "group_column": 'object_category',
                    "groups": c.OBJECTIVES,
                    "group_labels": c.OBJECTIVES_labels,
                    "ylabel": f"{label} (std)",
                    "xlabel": "Elapsed Time (min)",
                    # only show x-labels on bottom row; this gets overridden in helper
                    "show_xlabel": False,
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 30,
                    "grid": True,
                    "show_legend": True,     # suppressed inside when shared_legend=True
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                }
            )

        for metric, label in zip(metrics, metrics_label):
            # Row 2: mean
            subplot_cfgs.append(
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'mean_{metric}',
                    "group_column": 'object_category',
                    "groups": c.OBJECTIVES,
                    "group_labels": c.OBJECTIVES_labels,
                    "ylabel": f"{label} (mean)",
                    "xlabel": "Elapsed Time (min)",
                    "show_xlabel": True,     # bottom row – will keep labels
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 30,
                    "grid": True,
                    "show_legend": True,     # suppressed inside when shared_legend=True
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                }
            )

        # 6 subplots: 2 rows × 3 columns
        output_path = os.path.join(output_folder, f"{test}_all_metrics.pdf")

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=(9, 4),              # width for 3 side-by-side plots
            sharex=True,
            sharey=False,
            x_label_on_bottom_only=True,
            shared_legend=True,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.68, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
            grid_shape=(2, 3),            # <<< NEW: 2 rows × 3 columns
        )


def create_obj_compare_request_boxplot_triple(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')


    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        # Preprocess data (same as before)

        processed_request_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='request',
            value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            filter_condition=lambda df: df[df['TripDelay'] >= 0],
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['object_category', 'Instance_category', 'object_category2']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        # Save to temporary file

        processed_request_df.to_csv(request_path, index=False)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['object_category', 'Instance_category', 'object_category2']
        )

        # Save to temporary file

        processed_time_df.to_csv(time_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'WaitTime',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.25,
            'show_outliers': outlier,
            'ylim': (0,37),
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip delay
  #          'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.25,
            'ylim': (0, 8.4),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'nbRequests',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.25,
            'ylim': (0, 2600),
            'show_outliers': outlier,
            'show_legend': False,
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.66, 1),
        'ncol': 3,
        'title': 'Object Settings'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'ObjCompare_wait_delay_request_boxplot_{outlier}.pdf',
        fig_size=(9.5, 4),
        n_rows=1,
        n_cols=3,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.87],
    )

    # Clean up
    #if os.path.exists(request_path):
    #    os.remove(request_path)
    #    os.remove(time_path)

    return figure_path

def create_obj_compare_request_boxplot_fourth(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')


    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        # Preprocess data (same as before)

        processed_request_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='request',
            value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            filter_condition=lambda df: df[df['TripDelay'] >= 0],
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['object_category', 'Instance_category', 'object_category2']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        # Save to temporary file

        processed_request_df.to_csv(request_path, index=False)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['object_category', 'Instance_category', 'object_category2']
        )

        # Save to temporary file

        processed_time_df.to_csv(time_path, index=False)

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'WaitTime',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.3,
            'show_outliers': outlier,
            'ylim': (0,37),
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip delay
  #          'data_path': request_path,
            'data_df': processed_request_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.3,
            'ylim': (0, 8.4),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 3. nbRequests
      #      'data_path': time_path,  # <--- per-subplot
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'nbRequests',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.3,
            'ylim': (0, 2600),
            'show_outliers': outlier,
            'show_legend': False,
        },
        {  # 2: Trip delay
            #          'data_path': request_path,
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'object_category2',
            'value_column': 'EpochRuntime',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
         #   'ylim': (0, 78),
            #          'ylim': (0, 131),
            'rotation': 15,
            'target_lines': 30,
            'width': 0.3,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1),
        'ncol': 4,
        'title': 'Object Settings'
    }
    shared_target_line_legend = {
        'value': 30,
        'label': f'Epoch size (30s)',
        'bbox': (0.315, 0.9)
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'ObjCompare_wait_delay_request_time_boxplot_{outlier}.pdf',
        fig_size=(13, 4),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

    # Clean up
    #if os.path.exists(request_path):
    #    os.remove(request_path)
    #    os.remove(time_path)

    return figure_path

def create_obj_compare_vehicle_KPI_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'vehicle_temp_data.csv')
    # Preprocess data (same as before)
    KPI_columns_left = ['idleTime', 'driveFullTime']
    kpi_name_map_left = {
        'idleTime': 'Idle Time',
        'driveFullTime': 'Drive Full',
    }

    if os.path.exists(time_path):
        processed_df = read_csv_with_encoding(time_path)

    else:
        main_df = read_csv_with_encoding(data_path)

        processed_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='vehicle',
            value_column=['idleTime', 'driveEmptyTime', 'driveFullTime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['object_category', 'Instance_category', 'object_category2']
        )


        processed_df.to_csv(time_path, index=False)

    df_left = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=KPI_columns_left,
        kpi_name_map=kpi_name_map_left,
        category_column='object_category2',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )
    df_right = build_vehicle_kpi_long_df(
        nested_df=processed_df,
        value_columns=['driveEmptyTime'],
        kpi_name_map={'driveEmptyTime': 'Drive Empty'},
        category_column='object_category2',
        item_column_name="KPI",
        value_col_name="value",
        extra_id_columns=['Instance', 'Algorithm'],  # optional
    )

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Idle Times
            "data_df": df_left,
            'item_column': 'KPI',
            'category_column': 'object_category2',
            'value_column': 'value',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Times (min)',
            'xlabel': '       Vehicle KPIs',
            'rotation': 15,
            'width': 0.25,
            'ylim': (0,125),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Drive Empty
            "data_df": df_right,
            'item_column': 'KPI',
            'category_column': 'object_category2',
            'value_column': 'value',
            'categories': c.OBJECTIVES2,
            'category_labels': c.OBJECTIVES_labels2,
            'ylabel': 'Times (min)',
            'xlabel': ' ',
            'rotation': 15,
            'width': 0.25,
            'ylim': (0, 29),
            'show_outliers': outlier,
            'show_ylabel': False,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'ObjCompare_driving_KPI_boxplot_{outlier}.pdf',
        fig_size=(3,4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0.02, 1, 0.88),
    )

    return figure_path

def create_obj_compare_multi_std_plot(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)

    main_folder_path = os.path.join(base_path, 'vehicle_epoch_plots')
    os.makedirs(main_folder_path, exist_ok=True)

    # We will produce ONE figure per test, containing all metrics
    output_folder = os.path.join(main_folder_path, 'vehicle_epoch_all_metrics_plots')
    os.makedirs(output_folder, exist_ok=True)

    metrics = ['nbRequests', 'nbNodes', 'length']
    metrics_label = ['#Requests', '#Stops', 'length']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='epochVehicle',
        value_column=metrics,
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=['std', 'mean'],
        additional_columns=['object_category', 'Instance_category', 'object_category2']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_names = processed_df['Instance'].unique()

    palette = sns.color_palette("gist_earth", n_colors=4)
#    palette = palette12[::4]

    # ---- Loop over each test instance (ONE figure per test) ----
    for test in test_names:
        result_df = processed_df[processed_df['Instance'] == test]

        subplot_cfgs = []

        # We want a 2x3 grid: first row = std, second row = mean
        # columns: nbRequests | nbNodes | length
        for metric, label in zip(metrics, metrics_label):
            # Row 1: std
            subplot_cfgs.append(
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'std_{metric}',
                    "group_column": 'object_category2',
                    "groups": c.OBJECTIVES2,
                    "group_labels": c.OBJECTIVES_labels2,
                    "ylabel": f"{label} (std)",
                    "xlabel": "Elapsed Time (min)",
                    # only show x-labels on bottom row; this gets overridden in helper
                    "show_xlabel": False,
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 30,
                    "grid": True,
                    "show_legend": True,     # suppressed inside when shared_legend=True
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                }
            )

        for metric, label in zip(metrics, metrics_label):
            # Row 2: mean
            subplot_cfgs.append(
                {
                    "x_column": 'ElapsedTime',
                    "y_column": f'mean_{metric}',
                    "group_column": 'object_category2',
                    "groups": c.OBJECTIVES2,
                    "group_labels": c.OBJECTIVES_labels2,
                    "ylabel": f"{label} (mean)",
                    "xlabel": "Elapsed Time (min)",
                    "show_xlabel": True,     # bottom row – will keep labels
                    "show_ylabel": True,
                    "xlim": (0, 120),
                    "x_major_step": 30,
                    "grid": True,
                    "show_legend": True,     # suppressed inside when shared_legend=True
                    "legend_config": {"loc": "upper left", "ncol": 1},
                    "palette": palette,
                }
            )

        # 6 subplots: 2 rows × 3 columns
        output_path = os.path.join(output_folder, f"{test}_all_metrics2.pdf")

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=(13, 4),              # width for 3 side-by-side plots
            sharex=True,
            sharey=False,
            x_label_on_bottom_only=True,
            shared_legend=True,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 6,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
            grid_shape=(2, 3),            # <<< NEW: 2 rows × 3 columns
        )

def create_obj_compare_time_request_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'request_runtime_profiles')
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', 'Epoch'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['object_category', 'Instance_category', 'object_category2']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * 30) / 60
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=6)

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'EpochRuntime',
                "group_column": 'object_category2',
                "groups": c.OBJECTIVES2,
                "group_labels": c.OBJECTIVES_labels2,
                "ylabel": "Epoch time (s)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                #              "target_lines": 30,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbRequests',
                "group_column": 'object_category2',
                "groups": c.OBJECTIVES2,
                "group_labels": c.OBJECTIVES_labels2,
                "ylabel": "# Requests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 120),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_runtime_requests2.pdf"
        )

        create_multi_subplot_profile_lineplots(
            df=result_df,
            config=config,
            subplot_configs=subplot_cfgs,
            output_path=output_path,
            fig_size=(3,4),  # or config.fig_size_half
            sharex=True,  # share x-axis between subplots
            sharey=False,
            x_label_on_bottom_only=True,
            figure_title=None,
            shared_legend=False,
            shared_legend_config={
                "loc": "upper center",
                "bbox": (0.5, 0.98),
                "ncol": 3,
                "title": "Object Settings",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )

def create_custW3_barplot(data_path: str, config: PlotConfig) -> str:
    import os
    import numpy as np
    import seaborn as sns
    import matplotlib.pyplot as plt

    # 1) Read
    df = read_csv_with_encoding(data_path)

    # 2) (Optional) convert units if wait/cust is in seconds
    # If your column is already minutes, remove this.
    if "wait/cust" in df.columns:
        df["wait/cust"] = df["wait/cust"] / 60.0

    # 3) Ensure Req_W3 is consistently typed (THIS IS IMPORTANT)
    # Your categories are ["0","1"] so force strings.
    df["Req_W3"] = df["Req_W3"].astype(str)

    # 4) (Optional) enforce x-axis order if you have a preferred one
    # If you already have an ordering constant, use it here.
    # Example:
    # df["Instance_category"] = pd.Categorical(df["Instance_category"], categories=c.customer_groups, ordered=True)

    # 5) Pivot for grouped bars
    pivot = prepare_grouped_data(
        df,
        group_columns=["Instance_category", "Req_W3"],
        agg_column="wait/cust",
        agg_func="mean",
    )

    # Ensure category column order in the pivot
    categories = ["0", "1"]
    pivot = pivot.reindex(columns=[c for c in categories if c in pivot.columns])

    # 6) Labels for legend
    category_labels = [
        "Ignore Customer Weight",
        "Consider Customer Weight",
    ]
    categories_map = dict(zip(categories, category_labels))

    # 7) Colors (2 series)
    palette = sns.color_palette("gist_earth", n_colors=len(categories))
    alpha = 0.7
    palette_with_alpha = [(*color[:3], alpha) for color in palette]

    # 8) Plot
    fig, ax = plt.subplots(1, 1, figsize=(3.5, 4))

    _plot_grouped_bars(
        ax=ax,
        pivot_df=pivot,
        categories=categories_map,          # dict -> nice legend labels
        palette=palette_with_alpha,
        config=config,
        width=0.25,                        # thicker bars since only 2 per group
        cluster_spacing=1.0,               # reduce gaps between groups
        show_ylabel=True,
        ylabel="Avg. Waiting Time/Customer(min)",
        xlabel="Instance Category",
        rotation=15,
        log_scale=False,
        ylim=None,
        target_line=None,
        grid=False,
        xlim_padding=0.25
    )

    # 9) (Optional) add labels inside bars like your other plot
    ymin, ymax = ax.get_ylim()
    y_offset = (ymax - ymin) * 0.02
    label_fontsize = max(6, getattr(config, "tick_label_fsize", 9) - 2)

    for patch in ax.patches:
        height = patch.get_height()
        if np.isnan(height) or height <= 0:
            continue

        x = patch.get_x() + patch.get_width() / 2.0
        y = height - y_offset

        bar_color = patch.get_facecolor()
        text_color = (
            max(bar_color[0] - 0.4, 0),
            max(bar_color[1] - 0.4, 0),
            max(bar_color[2] - 0.4, 0),
            1.0
        )

        ax.text(
            x, y, f"{height:.2f}",
            ha="center", va="top",
            color=text_color,
            rotation=90,
            fontsize=label_fontsize,
            fontweight="bold",
        )

    # 11) Save
    fig.tight_layout()
    figure_path = os.path.join(os.path.dirname(data_path), "custW3_wait_cust_barplot.pdf")
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path
