import os
import constants as c
import seaborn as sns

from Simulation.utilities import read_csv_with_encoding
from Visualization.data_preprocessing import preprocess_nested_data
from Visualization.plot_config import PlotConfig
from Visualization.visualize_bar_plots import create_double_relative_barplot, create_double_grouped_barplot
from Visualization.visualize_box_plots import create_comparison_boxplots, create_single_boxplot, \
    create_multi_subplot_boxplots
from Visualization.visualize_profile_plots import create_multi_subplot_profile_lineplots
from Visualization.visualize_scatter_plots import plot_pruning_scatter_double, create_multi_subplot_lineplots, \
    create_grouped_lineplot

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
        ylabel='Total Runtime (s)',
        output_filename='pruning_time_boxplot.png',
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
        output_filename='dropPick_time_boxplot.png',
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


def create_dynamic_time_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for dynamic pricing comparison with drop-pick filter."""

    isDropPickPossible = True

    def filter_drop_pick(df):
        return df[df['isDropPickPossible'] == isDropPickPossible]

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
        output_filename=f'dynamic_time_boxplot_{isDropPickPossible}.png',
        width=0.55,
        target_lines=30,
        rotation=15,
        legend_bbox=(0.55, 0.99),  # bbox_anchor_legend1
        legend_bbox2=(0.43, 0.89),  # bbox_anchor_legend2
        legend_ncol=2,
        color_reverse=True,
        show_outliers=True,
        fig_size=config.fig_size,
        additional_filter=filter_drop_pick,
        tight_layout_rect=(0, 0, 1, 0.85)
    )


def create_dynamic_avgtime_boxplot_double(data_path: str, config: PlotConfig):
    """Create double boxplot for dynamic average time comparison."""

    isDropPickPossible = True

    # Preprocess function to calculate average time
    def preprocess_avgtime(df):
        df = df[df['isDropPickPossible'] == isDropPickPossible].copy()
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
        output_filename=f'dynamic_avgtime_boxplot_{isDropPickPossible}.png',
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
        df = df[(df['Ride_W2'] == 0.5) & (df['isTruncated'] == True)]
        return df

    return create_single_boxplot(
        data_path=data_path,
        config=config,
        item_column='MaxLabel',  # X-axis groups by MaxLabel
        category_column='sortPath',
        value_column='Total time',
        categories=['LAMBDA_SCORE', 'PATH_SCORE', 'REDUCED_COST'],
        category_labels=['Lambda Score', 'Normalized Reduced Cost', 'Reduced Cost'],
        legend_title='Sort Options',
        ylabel='Runtime (s)',
        xlabel='$M^{Label}$',
        output_filename='truncate_boxplot_05.png',
        color_reverse=True,
        width=0.5,
        target_lines={'Epoch Size (30s)': 30},
        rotation=0,
        additional_filter=preprocess_truncate,
        show_outliers=True
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
        output_filename=f'commit_responseTime_boxplot_{outlier}.png',
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
        output_filename=f'commit_waitTime_boxplot_{outlier}.png',
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


def create_ablation_wait_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
    main_df = main_df[main_df['Dynamic_Pricing'] == True]

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
        additional_columns=['Ride_W2', 'setting']
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
        category_column='setting',
        value_column='WaitTime',
        categories=['Full Configuration', 'Drop-Pick Allowed', 'Without Truncation'],
        category_labels=['Full Configuration', 'Drop-Pick Allowed', 'Without Truncation'],
        legend_title='Setting',
        ylabel='Waiting Times (min)',
        output_filename=f'ablation_waitTime_boxplot_{outlier}.png',
        width=0.35,
        rotation=15,
        legend_bbox=(0.5, 0.99),
        legend_ncol=2,
        color_reverse=True,
        show_outliers=outlier,
        tight_layout_rect=(0, 0, 1, 0.85),
        fig_size=config.fig_size
    )

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

    return figure_path


def create_ablation_time_boxplot_double(data_path: str, config: PlotConfig):
    # Read main data
    main_df = read_csv_with_encoding(data_path)

    # Create the 'setting' column by mapping paramFile values
    main_df['setting'] = main_df['paramFile'].map(c.param_to_setting)
    main_df = main_df[main_df['Dynamic_Pricing'] == True]
    # Preprocess commit wait time data

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column='EpochRuntime',
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,  # Keep all values for boxplot
        additional_columns=['Ride_W2', 'setting']
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
        category_column='setting',
        value_column='EpochRuntime',
        categories=['Full Configuration', 'Drop-Pick Allowed', 'Without Truncation'],
        category_labels=['Full Configuration', 'Drop-Pick Allowed', 'Without Truncation'],
        legend_title='Setting',
        ylabel='Epoch Runtime (s)',
        output_filename=f'ablation_runTime_boxplot_{outlier}.png',
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

    # Clean up
    if os.path.exists(temp_path):
        os.remove(temp_path)

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
        output_filename=f'multiObj_wait_delay_boxplot_{outlier}.png',
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
        output_filename=f'multiObj_response_assign_boxplot_{outlier}.png',
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
            'category_column': 'object_category',
            'value_column': 'TripDelay',
            'categories': c.OBJECTIVES,
            'category_labels': c.OBJECTIVES_labels,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Trip Delay',
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
        output_filename=f'multiObj_idle_empty_boxplot_{outlier}.png',
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
        output_filename=f'multiObj_full_delay_boxplot_{outlier}.png',
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
        output_filename=f'multiObj_size_time_boxplot_{outlier}.png',
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
        output_filename=f'multiObj_pass_req_boxplot_{outlier}.png',
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
            'category_column': 'Req_W3',
            'value_column': '#LGenerated',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated labels',
            'xlabel': 'Generated labels per Epoch',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'Instance_category',
            'category_column': 'Req_W3',
            'value_column': 'EpochRuntime',
            'categories': ["0", "1"],
            'category_labels': category_labels,
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
        {  # LEFT: Problem Size
            'item_column': 'Instance_category',
            'category_column': 'Req_W3',
            'value_column': 'totalRoutes',
            'categories': ["0", "1"],
            'category_labels': category_labels,
            'ylabel': '# Generated routes',
            'xlabel': 'Generated routes per Epoch',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # RIGHT: Epoch Runtime
            'item_column': 'Instance_category',
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
        output_filename=f'custW3_lable_time_boxplot_{outlier}.png',
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
        output_filename=f'custW3_route_dual_boxplot_{outlier}.png',
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


# ---------------------------------------------------------------------------
# Bar plots
# ---------------------------------------------------------------------------
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
        output_filename='dropPick_wait_barplot.png',
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
        output_filename='nbPick_wait_barplot.png',
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
        output_filename='nbPick_obj_barplot.png',
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
        output_filename='nbPick_time_barplot.png',
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
        category_labels=['No pruning', 'Prune nodes', 'Prune arcs', 'Prune Suboptimal Path'],
        output_filename='pruning_labels_comparison.png',
        ylabel='# Labels(Generated/LDominated)',
        ride_w2_values=(0.0, 0.5),
        log_scale=True,
        rotation=15,
        color_reverse=True,
        ylim=(2e6, 2e9)
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
        output_filename="multiObj_wait_lineplots.png",
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
        output_filename="multiObj_wait_grouped.png",
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
        output_filename="multiObj_tripdelay_grouped.png",
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
        output_filename="multiObj_passenger_grouped.png",
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
        output_filename="multiObj_idle_grouped.png",
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
        output_filename="custW3_wait_grouped.png",
        ylabel="waiting time per request(s)",
        category_labels=category_labels,
        group_labels=c.vehicle_groups_labels,
        legend_title="Customer Weight",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
        ylim=(102, 279),
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
        output_filename="custW3_waitcust_grouped.png",
        ylabel="waiting time per customer(s)",
        category_labels=category_labels,
        group_labels=c.vehicle_groups_labels,
        legend_title="Customer Weight",
        color_reverse=False,  # or True, as you prefer
        fig_size=(6.5, 4),
        ylim=(102, 279),
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
            f"{test}_runtime_requests.png"
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
            f"{test}_request_passengers.png"
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
