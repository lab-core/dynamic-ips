import os
import constants as c
import seaborn as sns

from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.data_preprocessing import preprocess_nested_data, build_vehicle_kpi_long_df
from Visualization.plot_config import PlotConfig
from Visualization.visualize_bar_plots import create_double_relative_barplot, create_double_grouped_barplot, \
    _plot_grouped_bars, prepare_grouped_data
from Visualization.visualize_box_plots import create_comparison_boxplots, create_single_boxplot, \
    create_multi_subplot_boxplots, plot_epoch_vehicle_boxplot, create_anytime_improve_boxplot_with_zoom, \
    create_anytime_improve_lineplot_with_zoom, create_anytime_epoch_count_barplot, \
    prepare_boxplot_data, calculate_positions, plot_boxplot_group
from Visualization.visualize_others import create_single_heatmap_figure, create_shaded_mean_std_plot
from Visualization.visualize_profile_plots import create_multi_subplot_profile_lineplots, plot_profile_line_group
from Visualization.visualize_scatter_plots import plot_pruning_scatter_double, create_multi_subplot_lineplots, \
    create_grouped_lineplot, plot_pruning_scatter_single
from Visualization.visualize_violin_plots import create_multi_subplot_violinplots
from Visualization.visualize_histogram import create_histograms_for_metrics

outlier = True


def create_rebalance_vehicle_boxplot_double(data_path: str, config: PlotConfig,
                                           params=None, param_labels=None) -> str:
    vehicle_path = os.path.join(os.path.dirname(data_path), 'vehicle_temp_data.csv')
    if os.path.exists(vehicle_path):
        processed_df = read_csv_with_encoding(vehicle_path)
    else:
        main_df = read_csv_with_encoding(data_path)
        processed_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='vehicle',
            value_column=['idleTime', 'driveFullTime', 'driveEmptyTime', 'returnEmptyTime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category']
        )
        processed_df.to_csv(vehicle_path, index=False)

    
    # Read main data
    data_df = processed_df
    paramFiles        = params       or ['Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2']
    paramFiles_labels = param_labels or ['No rebalance', 'Wait ≥ 6min', 'Wait ≥ 5min', 'Wait ≥ 4min', 'Wait ≥ 3min', 'Wait ≥ 2min']

    palette_6 = sns.color_palette("gist_earth", n_colors=6)[::-1]
    palette = ["", "", "", "", "", ""]
    palette[0] = palette_6[0]
    palette[1] = palette_6[2]
    palette[2] = palette_6[5]
    palette[3] = palette_6[4]
    palette[4] = palette_6[3]
    palette[5] = palette_6[1]



    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # LEFT: Idle Times
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'idleTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Idle Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.27,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': (0, 265),
    #        'ylim': (0, 149),
        },
        {  # RIGHT: Drive Empty
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'returnEmptyTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Rebalance Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.27,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
            'ylim': (0, 69),
  #          'ylim': (0, 29),
        }
    ]

    subplot_configs_2 = [
        {  # LEFT: Drive full
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'driveFullTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Drive full Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.27,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
            'ylim': (0, 235),
        },
        {  # RIGHT: Requests Served
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'driveEmptyTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Drive Empty Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.27,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
            'ylim': (0, 29),
    #        'ylim': (0, 24),
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.0),
        'ncol': 3,
        'title': 'Rebalancing Policy'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=vehicle_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'rebalance_idle_return_boxplot_{outlier}.pdf',
        fig_size=(8, 4),
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.88),
        palette=palette,
    )

    figure_path = create_multi_subplot_boxplots(
        data_path=vehicle_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'rebalance_full_empty_boxplot_{outlier}.pdf',
        fig_size=(8, 4),
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=(0, 0, 1, 0.88),
        palette=palette,
    )

    return figure_path

def create_rebalance_request_violinplot_double(data_path: str, config: PlotConfig) -> str:
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    if os.path.exists(request_path):
        processed_df = read_csv_with_encoding(request_path)
    else:
        main_df = read_csv_with_encoding(data_path)
        processed_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='request',
            value_column=['WaitTime', 'TripDelay'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            filter_condition=lambda df: df[df['TripDelay'] >= 0],
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category']
        )
        processed_df['TripDelay'] = processed_df['TripDelay'] - 0.5
        processed_df.to_csv(request_path, index=False)

    data_df = processed_df
    paramFiles = ['Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2']
    paramFiles_labels = ['No rebalance', 'Wait ≥ 6min', 'Wait ≥ 5min', 'Wait ≥ 4min', 'Wait ≥ 3min', 'Wait ≥ 2min']

    palette_6 = sns.color_palette("gist_earth", n_colors=6)[::-1]
    palette = ["", "", "", "", "", ""]
    palette[0] = palette_6[0]
    palette[1] = palette_6[2]
    palette[2] = palette_6[5]
    palette[3] = palette_6[4]
    palette[4] = palette_6[3]
    palette[5] = palette_6[1]

    subplot_configs_1 = [
        {  # 1: Waiting Times
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 0,
            'width': 0.27,
            'ylim': (0,22),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip delay
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Trip delay',
            'rotation': 0,
            'width': 0.27,
            'ylim': (0, 14.8),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.0),
        'ncol': 3,
        'title': 'Rebalancing Policy'
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=request_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'rebalance_wait_delay_violinplot_{outlier}.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
        palette=palette,
    )

    return figure_path

def create_rebalance_request_boxplot_double(data_path: str, config: PlotConfig,
                                           params=None, param_labels=None) -> str:
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')    
    if os.path.exists(request_path):
        processed_df = read_csv_with_encoding(request_path)
    else:
        main_df = read_csv_with_encoding(data_path)
        processed_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='request',
            value_column=['WaitTime', 'TripDelay'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            filter_condition=lambda df: df[df['TripDelay'] >= 0],
            transform_func=lambda x: x / 60,  # seconds -> minutes
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category']
        )
        processed_df['TripDelay'] = processed_df['TripDelay'] - 0.5
        processed_df.to_csv(request_path, index=False)

    data_df = processed_df
    paramFiles        = params       or ['Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2']
    paramFiles_labels = param_labels or ['No rebalance', 'Wait ≥ 6min', 'Wait ≥ 5min', 'Wait ≥ 4min', 'Wait ≥ 3min', 'Wait ≥ 2min']

    palette_6 = sns.color_palette("gist_earth", n_colors=6)[::-1]
    palette = ["", "", "", "", "", ""]
    palette[0] = palette_6[0]
    palette[1] = palette_6[2]
    palette[2] = palette_6[5]
    palette[3] = palette_6[4]
    palette[4] = palette_6[3]
    palette[5] = palette_6[1]


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Waiting Times
   #         'data_path': request_path,
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Waiting Times',
            'rotation': 0,
            'width': 0.27,
            'ylim': (0,22),
 #           'ylim': (0, 19),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip delay
  #          'data_path': request_path,
            'data_df': data_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': paramFiles,
            'category_labels': paramFiles_labels,
            'ylabel': 'Trip delay (min)',
            'xlabel': 'Trip delay',
            'rotation': 0,
            'width': 0.27,
            'ylim': (0, 14.8),
 #           'ylim': (0, 11),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.0),
        'ncol': 3,
        'title': 'Rebalancing Policy'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=request_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'rebalance_wait_delay_boxplot_{outlier}.pdf',
        fig_size=(8,4),
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.88],
        palette =palette,
    )

    return figure_path

def create_rebalance_histo_by_supply(data_path: str, config: PlotConfig,
                                    params=None, param_labels=None) -> str:
    import pandas as pd

    df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)

    # Palette and labels (logical order)
    palette_6 = sns.color_palette("gist_earth", n_colors=6)
    param_files_raw = params       or ['Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2']
    labels_raw      = param_labels or ['No rebalance', 'Wait ≥ 6min', 'Wait ≥ 5min', 'Wait ≥ 4min', 'Wait ≥ 3min', 'Wait ≥ 2min']
    palette_raw = [
        palette_6[0], palette_6[1], palette_6[2],
        palette_6[3], palette_6[4], palette_6[5],
    ]
    # Order by color luminance (light first, dark last) so darker series are drawn on top
    order_by_light = [0, 5, 1, 4, 3, 2]
    paramFiles = [param_files_raw[i] for i in order_by_light]
    labels = [labels_raw[i] for i in order_by_light]
    palette = palette_raw
    highlight_policies = {'Rebalance_no', 'Rebalance_6', 'Rebalance_5', 'Rebalance_4', 'Rebalance_3', 'Rebalance_2'}

    metric_configs = [
        ('IdleTime', 'Vehicle idle time (minutes)', 'idleTime_histo', 'upper right'),
        ('WorkingTime', 'Vehicle drive time (minutes)', 'driveTime_histo', 'upper left'),
        ('ActiveTime', 'Vehicle serving time (minutes)', 'servingTime_histo', 'upper left'),
    ]

    def _build_vehicle_df(meta_df, vehicle_file_name: str) -> pd.DataFrame:
        plot_data = []
        for vr in paramFiles:
            for _, row in meta_df[meta_df['paramFile'] == vr].iterrows():
                file_path = os.path.join(base_path, row['Test_Folder'], vehicle_file_name)
                if not os.path.exists(file_path):
                    continue
                test_data = pd.read_csv(file_path)
                test_data['IdleTime'] = test_data['idleTime'] / 60
                test_data['WorkingTime'] = (
                    test_data['driveFullTime'] + test_data['driveEmptyTime'] + test_data['returnEmptyTime']
                ) / 60
                test_data['ActiveTime'] = (test_data['driveFullTime'] + test_data['driveEmptyTime']) / 60
                plot_data.append(pd.DataFrame({
                    'IdleTime': test_data['IdleTime'],
                    'WorkingTime': test_data['WorkingTime'],
                    'ReturnTime': test_data['returnEmptyTime'] / 60,
                    'ActiveTime': test_data['ActiveTime'],
                    'ReturnPolicy': vr,
                    'Instance': row.get('Instance'),
                    'VehicleID': test_data['VehicleID']
                }))
        return pd.concat(plot_data, ignore_index=True) if plot_data else None

    # 1) Per vehicle category (saved in base_path/histo_by_category)
    histo_by_category_dir = os.path.join(base_path, 'histo_by_category')
    os.makedirs(histo_by_category_dir, exist_ok=True)
    for vehicle_cat in c.vehicle_groups:
        vehicle_filtered_df = df[df['Instance_category'] == vehicle_cat]
        result_df = _build_vehicle_df(vehicle_filtered_df, 'Vehicles_D_RT_CG.csv')
        if result_df is not None:
            create_histograms_for_metrics(
                data_df=result_df,
                config=config,
                output_dir=histo_by_category_dir,
                suffix=vehicle_cat,
                metric_configs=metric_configs,
                categories=paramFiles,
                category_labels=labels,
                palette=palette,
                category_column='ReturnPolicy',
                highlight_categories=highlight_policies,
                legend_order=param_files_raw,
                highlight_alpha=0.45,
                bin_width=5,
                log_scale=True,
                xtick_interval=30,
                legend_title='Rebalancing policy',
                legend_ncol=2,
                ylabel='Number of Vehicles',
                xlim_by_metric={'IdleTime': (0, 245)},
            )

    # 2) Overall (no vehicle_cat filter)
    overall_df = _build_vehicle_df(df, 'Vehicles_D_RT_CG.csv')
    if overall_df is not None:
        create_histograms_for_metrics(
            data_df=overall_df,
            config=config,
            output_dir=base_path,
            suffix='ALL',
            metric_configs=metric_configs,
            categories=paramFiles,
            category_labels=labels,
            palette=palette,
            category_column='ReturnPolicy',
            highlight_categories=highlight_policies,
            legend_order=param_files_raw,
            highlight_alpha=0.45,
            bin_width=5,
            log_scale=True,
            xtick_interval=30,
            legend_title='Rebalancing policy',
            legend_ncol=2,
            ylabel='Number of Vehicles',
            xlim_by_metric={'IdleTime': (0, 245)},
        )

    return "Histograms saved for all vehicle supply categories + overall (ALL)"


def create_process_violinplot(data_path: str, config: PlotConfig) -> str:
    """
    Plot MIP (integrality) gap as violin plot for algorithms A-CG-CR and B-CG-CR,
    grouped by Instance_category (vehicle supply).
    """
    import numpy as np

    main_df = read_csv_with_encoding(data_path)

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', 'Objective', 'LinearObjective'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Ride_W2', 'Algorithm_Name', 'Instance_category']
    )

    # MIP gap: 100 * (Objective - LinearObjective) / Objective (%)
    processed_time_df = processed_time_df.copy()
    processed_time_df['GAP'] = np.where(
        processed_time_df['Objective'] != 0,
        100.0 * (processed_time_df['Objective'] - processed_time_df['LinearObjective']) / processed_time_df['Objective'],
        np.nan,
    )
    processed_time_df = processed_time_df.dropna(subset=['GAP'])


    Algorithm_Names = ['A-CG-CR-SP1', 'A-CG-CR-SP2', 'B-CG-CR']
    Algorithm_Names_labels = ['A-CG-C (By Route)', 'A-CG-C (By Graph)', 'B-CG-C']

    gap_df = processed_time_df[processed_time_df['Algorithm_Name'].isin(Algorithm_Names)].copy()
    processed_time_df['Algorithm_Name'] = processed_time_df['Algorithm_Name'].astype(str).str.strip()


    palette_2 = sns.color_palette("gist_earth", n_colors=3)
    palette = list(palette_2[::-1])

    subplot_configs = [
        {
            'data_df': processed_time_df,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'GAP',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Integrality Gap (%)',
            'xlabel': 'Demand category',
            'rotation': 0,
            'width': 0.5,
            'ylim': (0, 0.17),
            'show_legend': False,
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 1,
        'title': 'Algorithm',
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs,
        output_filename='mip_gap_violinplot.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=1,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_stats_legend=True,
        shared_stats_legend_config={
            'loc': 'upper right',
            'bbox': (0.98, 0.98),
            'ncol': 1,
        },
        color_reverse=False,
        palette=palette,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.85],
    )
    return figure_path

def create_process_time_boxplot_three(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    Algorithm_Names = ['A-CG-CR-SP1', 'B-CG-CR', 'Greedy-CR']
    Algorithm_Names_labels = ['A-CG', 'B-CG', 'Greedy']

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 2: Waiting Times
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 14.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'TripDelay',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 15.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
            'ylim': (-0.4,31),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'Algorithm'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'process_three_boxplot_{outlier}.pdf',
        fig_size=(10, 4),
        n_rows=1,
        n_cols=3,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

    return figure_path

def create_process_wait_boxplot_two(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    palette = sns.color_palette("gist_earth", n_colors=3)

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    Algorithm_Names = ['A-CG-CR', 'B-CG-CR', 'Greedy-CR']
    Algorithm_Names_labels = ['A-CG', 'B-CG', 'Greedy']

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 2: Waiting Times
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'Algorithm_Name',
            'value_column': 'WaitTime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0,14.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels2,
            'category_column': 'Algorithm_Name',
            'value_column': 'TripDelay',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 15.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.75, 0.99),
        'ncol': 3,
        'title': 'Algorithm'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'process_two_boxplot_{outlier}.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

    return figure_path

def create_process_time_boxplot_two(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    palette = sns.color_palette("gist_earth", n_colors=3)

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'Algorithm'
    }

    Alg_part1 = ['A-CG-CR', 'Greedy-CR']
    Alg_part1_labels = ['A-CG-C', 'Greedy']

    Alg_part2 = ['B-CG-CR']
    Alg_part2_labels = ['B-CG-C']


    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Alg_part1,
            'category_labels': Alg_part1_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': '        Demand Category',
            'ylim': (-0.02, 0.62),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.5,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Alg_part2,
            'category_labels': Alg_part2_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': '',
            'ylim': (-0.4, 31),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.6,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
            'show_ylabel': False,
        },
    ]

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'process_two_time_boxplot_{outlier}.pdf',
        fig_size=(4.5,4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    return figure_path

def create_process_combined_boxplot(data_path: str, config: PlotConfig) -> str:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        processed_request_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='request',
            value_column=['WaitTime', 'TripDelay', 'CommitWaitTime', 'AssignTime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            filter_condition=lambda df: df[df['TripDelay'] >= 0],
            transform_func=lambda x: x / 60,
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_request = processed_request_df
    data_df_time = processed_time_df

    # Algorithm sets
    Algorithm_Names        = ['A-CG-CR',   'B-CG-CR',  'Greedy-CR']
    Algorithm_Names_labels = ['A-CG',      'B-CG',     'Greedy'   ]
    Alg_part1              = ['A-CG-CR',   'Greedy-CR']
    Alg_part1_labels       = ['A-CG-C',   'Greedy'   ]
    Alg_part2              = ['B-CG-CR']
    Alg_part2_labels       = ['B-CG-C']

    # 3-color palette: index 0→A-CG-CR, 1→B-CG-CR, 2→Greedy-CR (consistent across all panels)
    palette       = sns.color_palette("gist_earth", n_colors=3)
    palette_part1 = [palette[0], palette[2]]  # A-CG-CR and Greedy-CR
    palette_part2 = [palette[1]]              # B-CG-CR only

    # 6-column GridSpec: two silent spacer columns create equal 0.4-wide gaps between
    # panel 1–2 and panel 2–3, while wspace=0.2 keeps the right pair (panels 3–4) tight.
    gap_left = 0.4   # spacer between panels 1 and 2
    gap_mid  = 0.4   # spacer between the left pair and the right pair
    wspace   = 0.2   # uniform inter-column spacing (controls tightness of right pair)
    fig = plt.figure(figsize=(12, 4))
    gs = gridspec.GridSpec(1, 6, figure=fig,
                           width_ratios=[3.5, gap_left, 3.5, gap_mid, 3, 2],
                           wspace=wspace)
    ax1 = fig.add_subplot(gs[0, 0])
    # gs[0, 1] is the left spacer — intentionally left unused
    ax2 = fig.add_subplot(gs[0, 2])
    # gs[0, 3] is the middle spacer — intentionally left unused
    ax3 = fig.add_subplot(gs[0, 4])
    ax4 = fig.add_subplot(gs[0, 5])

    # Prepare columns as strings (required by prepare_boxplot_data)
    for df, cols in [
        (data_df_request, ['Instance_category', 'Algorithm_Name']),
        (data_df_time,    ['Instance_category', 'Algorithm_Name']),
    ]:
        for col in cols:
            df[col] = df[col].astype(str)

    x_req  = np.sort(data_df_request['Instance_category'].unique()).astype(str)
    x_time = np.sort(data_df_time['Instance_category'].unique()).astype(str)

    # Panel 1: Waiting Times
    all_handles, all_labels_out = plot_boxplot_group(
        ax=ax1,
        plot_data=prepare_boxplot_data(
            data_df_request, x_req, 'Instance_category',
            Algorithm_Names, 'Algorithm_Name', 'WaitTime'),
        categories=Algorithm_Names, palette=palette, config=config,
        show_outliers=outlier, show_ylabel=True,
        ylabel='Waiting Times (min)', width=0.35,
        xlabel='Demand Category', x_tick_labels=c.vehicle_groups_labels2,
        rotation=0, ylim=(0, 14.5),
        show_legend=False, category_labels=Algorithm_Names_labels,
    )

    # Panel 2: Trip Delay
    plot_boxplot_group(
        ax=ax2,
        plot_data=prepare_boxplot_data(
            data_df_request, x_req, 'Instance_category',
            Algorithm_Names, 'Algorithm_Name', 'TripDelay'),
        categories=Algorithm_Names, palette=palette, config=config,
        show_outliers=outlier, show_ylabel=True,
        ylabel='Trip Delay (min)', width=0.35,
        xlabel='Demand Category', x_tick_labels=c.vehicle_groups_labels2,
        rotation=0, ylim=(0, 15.5),
        show_legend=False, category_labels=Algorithm_Names_labels,
    )

    # Panel 3: Epoch Runtime – A-CG and Greedy (color-matched to panels 1 & 2)
    plot_boxplot_group(
        ax=ax3,
        plot_data=prepare_boxplot_data(
            data_df_time, x_time, 'Instance_category',
            Alg_part1, 'Algorithm_Name', 'EpochRuntime'),
        categories=Alg_part1, palette=palette_part1, config=config,
        show_outliers=outlier, show_ylabel=True,
        ylabel='Epoch Runtime (s)', width=0.5,
        xlabel='        Demand Category', x_tick_labels=c.vehicle_groups_labels2,
        rotation=0, ylim=(-0.02, 0.62),
        show_legend=False, category_labels=Alg_part1_labels,
    )

    # Panel 4: Epoch Runtime – B-CG only (color-matched to panels 1 & 2)
    plot_boxplot_group(
        ax=ax4,
        plot_data=prepare_boxplot_data(
            data_df_time, x_time, 'Instance_category',
            Alg_part2, 'Algorithm_Name', 'EpochRuntime'),
        categories=Alg_part2, palette=palette_part2, config=config,
        show_outliers=outlier, show_ylabel=False,
        ylabel='Epoch Runtime (s)', width=0.6,
        xlabel='', x_tick_labels=c.vehicle_groups_labels2,
        rotation=0, ylim=(-0.4, 31),
        target_lines=30,
        show_legend=False, category_labels=Alg_part2_labels,
    )

    # Single shared legend (handles from panel 1, which covers all 3 algorithms)
    fig.legend(
        handles=all_handles,
        labels=all_labels_out,
        loc='upper center',
        bbox_to_anchor=(0.5, 1.04),
        ncol=3,
        fontsize=config.legend_fsize,
        edgecolor=config.legend_edgecolor,
        title='Algorithm',
        title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
    )

    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.86])

    figure_path = os.path.join(os.path.dirname(data_path),
                               f'process_combined_boxplot_{outlier}.pdf')
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path

def create_process_percent_boxplot_two(data_path: str, config: PlotConfig) -> str:
    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(time_path):
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime','SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_time['Total_Time'] = data_df_time['MP_Runtime'] + data_df_time['SubProbRuntime']
    data_df_time['MP_Percent'] = 100 * data_df_time['MP_Runtime']/data_df_time['Total_Time']
    data_df_time['SP_Percent'] = 100 * data_df_time['SubProbRuntime'] / data_df_time['Total_Time']

    Algorithm_Names = ['A-CG-CR-SP1', 'A-CG-CR-SP2', 'B-CG-CR']
    Algorithm_Names_labels = ['A-CG-C (By Route)', 'A-CG-C (By Graph)', 'B-CG-C']

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
            #          'ylim': (0,68),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'SP_Percent',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': '% SP Time',
            'xlabel': 'Demand Category',
            #          'ylim': (0,68),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.37,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'Algorithm'
    }
    palette = sns.color_palette("gist_earth", n_colors=4)

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'process_percent_boxplot_{outlier}.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    return figure_path

def create_reOptimize_boxplot_fourth(data_path: str, config: PlotConfig) -> str:
    # Read main data
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    plot = True
    Param_Pool = ['Baseline_Pool', 'SP_Re_2_Pool', 'SP_Re_1_Pool']
    Param_No_Pool = ['Baseline', 'SP_Re_2', 'SP_Re_1']
    param = Param_Pool

   # data_df_time = data_df_time[data_df_time['Route_Recycle'] == plot]
   # data_df_request = data_df_request[data_df_request['Route_Recycle'] == plot]

    # Create the 'setting' column by mapping paramFile values
    SE_Methods = ['No SP Re-optimization', 'SP Re-optimization By Graph', 'SP Re-optimization By Route' ]


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'EpochRuntime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
  #          'ylim': (0,68),
  #          'ylim': (0, 305),
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 2: Waiting Times
   #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay  
   #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'nbRequests',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
  #          'ylim': (0,68),
  #          'ylim': (0, 305),
            'rotation': 15,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        
    ]


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'SP Re-optimization Method'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'reOptimize_{plot}_fourth_boxplot_{outlier}.pdf',
        fig_size=(13, 4),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

    return figure_path


def create_reOptimize_penalty_boxplot_fourth(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    palette = sns.color_palette("gist_earth", n_colors=4)[::-1]

    plot = False
    Param_Pool = ['Penalty','Baseline_Pool', 'SP_Re_2_Pool', 'SP_Re_1_Pool']
    Param_No_Pool = ['Penalty','Baseline', 'SP_Re_2', 'SP_Re_1']
    param = Param_No_Pool

 #   data_df_time = data_df_time[data_df_time['Route_Recycle'] == plot]
 #   data_df_request = data_df_request[data_df_request['Route_Recycle'] == plot]

    # Create the 'setting' column by mapping paramFile values
    SE_Methods = ['Penalty duals (No reduction)', 'Last-LP duals (No reduction)', 'Last-LP duals (Pool-coverage pruning)', 'Last-LP duals (Route-coverage pruning)']


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'EpochRuntime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
            'ylim': (0,5.8),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 2: Waiting Times
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 16.4),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 14.8),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'nbRequests',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            #          'ylim': (0,68),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 1050),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },

    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 4,
        'title': 'Initialization'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'reOptimize_{plot}_Penalty_boxplot_{outlier}.pdf',
        fig_size=(13, 4),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    return figure_path

def create_reOptimize_boxplot_triple(data_path: str, config: PlotConfig,
                                    params_no_pool=None, params_keep_pool=None,
                                    param_labels=None) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    palette = sns.color_palette("gist_earth", n_colors=4)[::-1]

    No_Pool   = params_no_pool   or ['Full_cold', 'Full_warm',      'Pool_warm',      'Basis_warm'     ]
    Keep_Pool = params_keep_pool or ['Full_cold', 'Full_warm_keep', 'Pool_warm_keep', 'Basis_warm_keep']

    plot_cases = [
        {
            "plot": False,
            "param": No_Pool,
            "name": "no_Pool",
        },
        {
            "plot": True,
            "param": Keep_Pool,
            "name": "keep_Pool",
        },
    ]

    SE_Methods = param_labels or [
        'Full network (dual cold-start)',
        'Full network (dual warm-start)',
        'Reduction by pool (dual warm-start)',
        'Reduction by basis (dual warm-start)',
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 4,
        'title': 'Network (Dual Strategy)'
    }

    figure_paths = []

    for case in plot_cases:
        plot = case["plot"]
        param = case["param"]
        name = case["name"]

        subplot_configs_1 = [
            {
                'data_df': data_df_time,
                'item_column': 'Instance_category',
                'x_tick_labels': c.vehicle_groups_labels2,
                'category_column': 'paramFile',
                'value_column': 'EpochRuntime',
                'categories': param,
                'category_labels': SE_Methods,
                'ylabel': 'Epoch Runtime (s)',
                'xlabel': 'Demand Category',
                'ylim': (0, 5.3),
                'rotation': 0,
                'width': 0.39,
                'show_outliers': outlier,
                'show_legend': False,
            },
            {
                'data_df': data_df_request,
                'item_column': 'Instance_category',
                'x_tick_labels': c.vehicle_groups_labels2,
                'category_column': 'paramFile',
                'value_column': 'WaitTime',
                'categories': param,
                'category_labels': SE_Methods,
                'ylabel': 'Waiting Times (min)',
                'xlabel': 'Demand Category',
                'rotation': 0,
                'width': 0.39,
                'ylim': (0, 16.4),
                'show_outliers': outlier,
                'show_legend': False,
            },
            {
                'data_df': data_df_request,
                'item_column': 'Instance_category',
                'x_tick_labels': c.vehicle_groups_labels2,
                'category_column': 'paramFile',
                'value_column': 'TripDelay',
                'categories': param,
                'category_labels': SE_Methods,
                'ylabel': 'Trip Delay (min)',
                'xlabel': 'Demand Category',
                'rotation': 0,
                'width': 0.39,
                'ylim': (0, 14.2),
                'show_outliers': outlier,
                'show_legend': False,
            },
        ]

        figure_path = create_multi_subplot_boxplots(
            data_path=data_path,
            config=config,
            subplot_configs=subplot_configs_1,
            output_filename=f'reOptimize_{plot}_triple_boxplot_{outlier}.pdf',
            fig_size=(11, 4),
            n_rows=1,
            n_cols=3,
            shared_legend=True,
            shared_legend_config=shared_legend_config,
            additional_filter=None,
            color_reverse=False,
            tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
            palette=palette,
        )

        figure_paths.append(figure_path)

    return figure_paths


def create_reOptimize_penalty_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    palette = sns.color_palette("gist_earth", n_colors=5)[::-1]

    plot = True
    Param_Pool = ['Penalty','Baseline_Pool', 'SP_Re_2_Pool', 'SP_Re_3_Pool', 'SP_Re_1_Pool']
    Param_No_Pool = ['Penalty','Baseline', 'SP_Re_2', 'SP_Re_3', 'SP_Re_1']
    param = Param_Pool

 #   data_df_time = data_df_time[data_df_time['Route_Recycle'] == plot]
 #   data_df_request = data_df_request[data_df_request['Route_Recycle'] == plot]

    # Create the 'setting' column by mapping paramFile values
    SE_Methods = ['full network', 'full network + dual warm-start', 'Pool pruning + dual warm-start', 'base pruning + dual warm-start', 'Route pruning + dual warm-start']


    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Runtime
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'EpochRuntime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Demand Category',
            'ylim': (0,5.8),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 2: Requests
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'nbRequests',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': '# Pending Requests',
            'xlabel': 'Demand Category',
            #          'ylim': (0,68),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 1050),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },

    ]

    subplot_configs_2 = [
        {  # 1: Waiting Times
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 16.4),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay
            'data_df': data_df_request,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': param,
            'category_labels': SE_Methods,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Demand Category',
            'rotation': 0,
            'width': 0.35,
            'ylim': (0, 14.8),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 2,
        'title': 'Dual/Network Strategy'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'reOptimize_{plot}_time_boxplot_{outlier}.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_2,
        output_filename=f'reOptimize_{plot}_quality_boxplot_{outlier}.pdf',
        fig_size=config.fig_size,
        n_rows=1,
        n_cols=2,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    return figure_path

def create_anytime_time_request_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df["Dynamic_Pricing"] == False]
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, 'request_runtime_profiles')
    os.makedirs(folder_path, exist_ok=True)

    Param_Dynamic = ['Iter_Dynamic_4', 'Iter_Dynamic_3', 'Iter_Dynamic_2', 'Iter_Dynamic_1']
    Param_Partial = ['Iter_Partial_4', 'Iter_Partial_3', 'Iter_Partial_2', 'Iter_Partial_1']
    Labels = ['4 Iteration', '3 Iteration', '2 Iteration', '1 Iteration']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', 'Epoch', 'ElapsedTime','waitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['paramFile', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['ElapsedTime']) / 60
    processed_df['waitTime'] = (processed_df['waitTime']) / processed_df['nbRequests']
    test_name = processed_df['Instance'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        palette = sns.color_palette("gist_earth", n_colors=4)[::-1]

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'EpochRuntime',
                "group_column": 'paramFile',
                "groups": Param_Partial,
                "group_labels": Labels,
                "ylabel": "Epoch time (s)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbRequests',
                "group_column": 'paramFile',
                "groups": Param_Partial,
                "group_labels": Labels,
                "ylabel": "# Pending requests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                "linewidth": 0.8,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_P_runtime_requests.pdf"
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
                "ncol": 4,
                "title": "# CG Iterations",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )


def create_rebalance_anytime_time_request_profile(data_path: str, config: PlotConfig,
                                                 params_left=None, params_right=None,
                                                 param_labels=None) -> None:
    import matplotlib.pyplot as plt

    Param_Pool  = params_left  or ['Pool_warm_keep',  'no_rebalance_Pool' ]
    Param_Basis = params_right or ['Basis_warm_keep', 'no_rebalance_Basis']
    Labels      = param_labels or ['With Rebalancing', 'Without Rebalancing']

    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df["ReOptimizeStrategy"] == 'BY_GRAPH']
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, "combined_rebalance_profiles")
    os.makedirs(folder_path, exist_ok=True)

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', 'Epoch', 'ElapsedTime', 'waitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['paramFile', 'Instance_category']
    )

    processed_df['ElapsedTime'] = (processed_df['ElapsedTime']) / 60
    processed_df['waitTime'] = (processed_df['waitTime']) / processed_df['nbRequests']
    test_name = processed_df['Instance'].unique()

    palette     = sns.color_palette("gist_earth", n_colors=2)[::-1]
    y_columns   = ['EpochRuntime', 'nbRequests']
    y_labels    = ['Epoch time (s)', '# Pending requests']
    linewidths  = [1.0, 0.8]

    orig_w, orig_h = config.fig_size
    combined_fig_size = (orig_w * 1.8, orig_h + 1)

    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        # Shared ylim per row: lower bound fixed, upper bound = max across both columns
        subset       = result_df[result_df['paramFile'].isin(Param_Pool + Param_Basis)]
        max_epoch    = subset['EpochRuntime'].max()
        max_requests = subset['nbRequests'].max()
        ylims = [(-2, max_epoch), (-50, max_requests)]

        # 2 rows × 2 columns; x shared within each column → only bottom row shows tick labels
        fig, axes = plt.subplots(2, 2, figsize=combined_fig_size, sharex='col', sharey=False)

        all_handles, all_labels_out = [], []

        for col_idx, param in enumerate([Param_Pool, Param_Basis]):
            for row_idx, (y_col, y_lbl, lw, ylim) in enumerate(zip(y_columns, y_labels, linewidths, ylims)):
                ax = axes[row_idx, col_idx]
                is_bottom = (row_idx == 1)

                handles, labels_out = plot_profile_line_group(
                    ax=ax,
                    df=result_df,
                    x_column='ElapsedTime',
                    y_column=y_col,
                    group_column='paramFile',
                    groups=param,
                    config=config,
                    palette=palette,
                    group_labels=Labels,
                    xlabel='Elapsed Time (min)',
                    ylabel=y_lbl,
                    show_xlabel=is_bottom,
                    show_ylabel=True,
                    xlim=(0, 240),
                    x_major_step=30,
                    grid=True,
                    show_legend=False,
                    linewidth=lw,
                    ylim=ylim,
                )

                if not is_bottom:
                    ax.tick_params(labelbottom=False)

                if col_idx == 0 and row_idx == 0:
                    all_handles, all_labels_out = handles, labels_out

        fig.legend(
            all_handles, all_labels_out,
            loc='upper center',
            bbox_to_anchor=(0.5, 0.98),
            ncol=2,
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title='Rebalancing',
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
        )

        fig.tight_layout(rect=[0, 0.1, 1, 0.86])

        left_pos  = axes[1, 0].get_position()
        right_pos = axes[1, 1].get_position()
        label_y   = min(left_pos.y0, right_pos.y0) - 0.14

        fig.text((left_pos.x0 + left_pos.x1) / 2, label_y,
                 '(a) Pool-based network reduction',
                 ha='center', va='top', fontsize=config.axis_title_fsize)
        fig.text((right_pos.x0 + right_pos.x1) / 2, label_y,
                 '(b) Basis-based network reduction',
                 ha='center', va='top', fontsize=config.axis_title_fsize)

        output_path = os.path.join(folder_path, f"{test}_combined_rebalance_requests.pdf")
        fig.savefig(output_path, bbox_inches='tight')
        plt.close(fig)


def create_reOptimize_time_request_profile(data_path: str, config: PlotConfig,
                                          params_left=None, params_right=None,
                                          param_labels=None) -> None:
    import matplotlib.pyplot as plt

    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(time_path):
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_time_df.to_csv(time_path, index=False)

    No_Pool   = params_left  or ['Full_cold', 'Full_warm',      'Pool_warm',      'Basis_warm'     ]
    Keep_Pool = params_right or ['Full_cold', 'Full_warm_keep', 'Pool_warm_keep', 'Basis_warm_keep']
    Labels = param_labels or [
        'Full network (dual cold-start)',
        'Full network (dual warm-start)',
        'Reduction by pool (dual warm-start)',
        'Reduction by basis (dual warm-start)',
    ]

    palette = sns.color_palette("gist_earth", n_colors=4)[::-1]

    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, "combined_runtime_profiles")
    os.makedirs(folder_path, exist_ok=True)

    processed_df = processed_time_df
    processed_df['ElapsedTime'] = (processed_df['ElapsedTime']) / 60
    test_name = processed_df['Instance'].unique()

    y_columns  = ['EpochRuntime', 'nbRequests']
    y_labels   = ['Epoch time (s)', '# Pending requests']
    linewidths = [0.8, 0.8]

    # Same sizing convention as create_reOptimize_dual_profile
    orig_w, orig_h = config.fig_size
    combined_fig_size = (orig_w * 1.8, orig_h + 1)

    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        # 2 rows × 2 columns; share x within each column → only bottom row shows tick labels
        fig, axes = plt.subplots(2, 2, figsize=combined_fig_size, sharex='col', sharey=False)

        all_handles, all_labels_out = [], []

        for col_idx, param in enumerate([No_Pool, Keep_Pool]):
            for row_idx, (y_col, y_lbl, lw) in enumerate(zip(y_columns, y_labels, linewidths)):
                ax = axes[row_idx, col_idx]
                is_bottom = (row_idx == 1)

                handles, labels_out = plot_profile_line_group(
                    ax=ax,
                    df=result_df,
                    x_column='ElapsedTime',
                    y_column=y_col,
                    group_column='paramFile',
                    groups=param,
                    config=config,
                    palette=palette,
                    group_labels=Labels,
                    xlabel='Elapsed Time (min)',
                    ylabel=y_lbl,
                    show_xlabel=is_bottom,
                    show_ylabel=True,
                    xlim=(0, 240),
                    x_major_step=30,
                    grid=True,
                    show_legend=False,
                    linewidth=lw,
                )

                if not is_bottom:
                    ax.tick_params(labelbottom=False)

                if col_idx == 0 and row_idx == 0:
                    all_handles, all_labels_out = handles, labels_out

        fig.legend(
            all_handles, all_labels_out,
            loc='upper center',
            bbox_to_anchor=(0.5, 0.98),
            ncol=2,
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title='Network (Dual Strategy)',
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
        )

        fig.tight_layout(rect=[0, 0.1, 1, 0.86])

        left_pos  = axes[1, 0].get_position()
        right_pos = axes[1, 1].get_position()
        label_y   = min(left_pos.y0, right_pos.y0) - 0.14

        fig.text((left_pos.x0 + left_pos.x1) / 2, label_y,
                 '(a) Discarding the LP basis and route pool',
                 ha='center', va='top', fontsize=config.axis_title_fsize)
        fig.text((right_pos.x0 + right_pos.x1) / 2, label_y,
                 '(b) Retaining the LP basis and column pool',
                 ha='center', va='top', fontsize=config.axis_title_fsize)

        output_path = os.path.join(folder_path, f"{test}_combined_runtime_requests.pdf")
        fig.savefig(output_path, bbox_inches='tight')
        plt.close(fig)

def create_reOptimize_dual_profile(data_path: str, config: PlotConfig,
                                  params_left=None, params_right=None,
                                  param_labels=None) -> None:
    import matplotlib.pyplot as plt

    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(time_path):
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category']
        )

        processed_time_df.to_csv(time_path, index=False)

    Param_Pool  = params_left  or ['Pool_warm',  'Pool_warm_keep' ]
    Param_Basis = params_right or ['Basis_warm', 'Basis_warm_keep']
    Labels      = param_labels or ['Discard Basis & pool ', 'Keep Basis & Column pool']

    palette = sns.color_palette("gist_earth", n_colors=4)
    palette = [palette[0], palette[2]][::-1]

    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, "combined_dual_profiles")
    os.makedirs(folder_path, exist_ok=True)

    processed_df = processed_time_df
    processed_df['ElapsedTime'] = (processed_df['ElapsedTime']) / 60
    test_name = processed_df['Instance'].unique()

    y_columns = ['maxDual', 'meanDual', 'minDual']
    y_labels = ['Max Dual', 'Mean Dual', 'Min Dual']
    linewidths = [1.2, 0.8, 0.8]

    # Combined figure: double the original width, same height
    orig_w, orig_h = config.fig_size
    combined_fig_size = (orig_w * 1.8, orig_h+1)

    for test in test_name:
        result_df = processed_df[processed_df['Instance'] == test]

        # 3 rows × 2 columns; share x within each column so only bottom row shows tick labels
        fig, axes = plt.subplots(3, 2, figsize=combined_fig_size, sharex='col', sharey=False)

        all_handles, all_labels_out = [], []

        for col_idx, param in enumerate([Param_Pool, Param_Basis]):
            for row_idx, (y_col, y_lbl, lw) in enumerate(zip(y_columns, y_labels, linewidths)):
                ax = axes[row_idx, col_idx]
                is_bottom = (row_idx == 2)

                handles, labels_out = plot_profile_line_group(
                    ax=ax,
                    df=result_df,
                    x_column='ElapsedTime',
                    y_column=y_col,
                    group_column='paramFile',
                    groups=param,
                    config=config,
                    palette=palette,
                    group_labels=Labels,
                    xlabel='Elapsed Time (min)',
                    ylabel=y_lbl,
                    show_xlabel=is_bottom,
                    show_ylabel=True,
                    xlim=(0, 240),
                    x_major_step=30,
                    grid=True,
                    show_legend=False,
                    linewidth=lw,
                )

                if not is_bottom:
                    ax.tick_params(labelbottom=False)

                if col_idx == 0 and row_idx == 0:
                    all_handles, all_labels_out = handles, labels_out

        # Single shared legend at the top (same categories and colors in both columns)
        fig.legend(
            all_handles, all_labels_out,
            loc='upper center',
            bbox_to_anchor=(0.5, 0.98),
            ncol=2,
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title='Reuse Information',
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
        )

        fig.tight_layout(rect=[0, 0.1, 1, 0.86])

        # Column labels below each column's bottom subplot
        left_pos = axes[2, 0].get_position()
        right_pos = axes[2, 1].get_position()
        label_y = min(left_pos.y0, right_pos.y0) - 0.14

        fig.text((left_pos.x0 + left_pos.x1) / 2, label_y,
                 '(a) Pool-based network reduction', ha='center', va='top',
                 fontsize=config.axis_title_fsize)
        fig.text((right_pos.x0 + right_pos.x1) / 2, label_y,
                 '(b) Basis-based network reduction', ha='center', va='top',
                 fontsize=config.axis_title_fsize)

        output_path = os.path.join(folder_path, f"{test}_combined_dual_route.pdf")
        fig.savefig(output_path, bbox_inches='tight')
        plt.close(fig)


def create_process_violinplot_double(data_path: str, config: PlotConfig):
    # Read main data

    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(time_path):
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_time['MP_Percent'] = 100 * data_df_time['MP_Runtime'] / data_df_time['EpochRuntime']
    data_df_time['SP_Percent'] = 100 * data_df_time['SubProbRuntime'] / data_df_time['EpochRuntime']

    Algorithm_Names = ['A-CG-CR-SP1', 'A-CG-CR-SP2', 'B-CG-CR']
    Algorithm_Names_labels = ['A-CG-C (By Route)', 'A-CG-C (By Graph)', 'B-CG-C']

    subplot_configs_1 = [
        {  # LEFT: Problem Size
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'MP_Percent',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': '% MP Time',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.42,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # Right: Problem Size
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'SP_Percent',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': '% SP Time',
            'xlabel': 'Demand Category',
            'rotation': 15,
            'width': 0.42,
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
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
        output_filename=f'process_percent_violin_{outlier}.pdf',
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


def create_anytime_improve_boxplot(data_path: str, config: PlotConfig) -> str:
    def preprocess_anytime(df):
        df = df[(df['subProIter'] < 5)]
        df = df[df['Model'].isin(['CG'])]
        return df

    figure_path = create_anytime_improve_boxplot_with_zoom(
        data_path=data_path,
        config=config,
        outlier=outlier,
        output_filename=f"anytime_boxplot_zoom_{outlier}.pdf",
        additional_filter=preprocess_anytime,
        inset_bounds=(0.38, 0.25, 0.45, 0.45),
        inset_ylim=(-0.02, 0.34),
  #      inset_ylim=(-0.02, 3.2),
        ylim=(-0.3, 34),
        inset_xpad=0.35,
        item_column='subProIter',
        category_column='epochLength',
        value_column='RelativeImprove',
        ylabel='Relative Improve (%)',
        categories=['30', '20', '10', '5'],
        box_width=0.35,
        xlabel= 'CG Iterations',
    )

def create_profile_boxplot(data_path: str, config: PlotConfig) -> str:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    def preprocess_anytime(df):
        df = df[(df['subProIter'] < 5)]
        df = df[df['Model'].isin(['CG'])]
        return df

    # --- Load data ---
    df_zoom = read_csv_with_encoding(data_path)
    df_zoom = preprocess_anytime(df_zoom)

    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[(main_df['subProIter'] < 5)]
    main_df = main_df[main_df['Model'].isin(['CG'])]

    # --- Build combined figure ---
    # 4 columns: [left panel, spacer, right1, right2]
    # The spacer column creates extra visual gap between left and right groups.
    fig = plt.figure(figsize=(13, 4.5))
    gs = gridspec.GridSpec(1, 4, figure=fig, width_ratios=[1.2, 0.1, 1, 1], wspace=0.15)
    ax_left = fig.add_subplot(gs[0, 0])
    ax_right1 = fig.add_subplot(gs[0, 2])
    ax_right2 = fig.add_subplot(gs[0, 3])

    # ==================================================
    # LEFT PANEL: Relative Improvement with inset zoom
    # ==================================================
    item_col = 'subProIter'
    cat_col = 'epochLength'
    val_col = 'RelativeImprove'
    categories = ['30', '20', '10', '5']
    box_width = 0.35

    df_left = df_zoom.copy()
    df_left[item_col] = df_left[item_col].astype(str)
    df_left[cat_col] = df_left[cat_col].astype(str)
    x_items = np.sort(df_left[item_col].unique()).astype(str)

    plot_data_left = prepare_boxplot_data(df_left, x_items, item_col, categories, cat_col, val_col)
    palette_left = sns.color_palette("gist_earth", n_colors=4)

    plot_boxplot_group(
        ax=ax_left,
        plot_data=plot_data_left,
        categories=categories,
        palette=palette_left,
        config=config,
        show_outliers=outlier,
        show_ylabel=True,
        ylabel='Relative Improve (%)',
        width=box_width,
        xlabel='CG Iterations',
        x_tick_labels=x_items,
        rotation=0,
        ylim=(-0.3, 34),
        show_legend=True,
        legend_loc='upper right',
        legend_title='Epoch size (s)',
        legend_ncol=2,
    )

    # Inset zoom on groups 1 and 2
    base_pos, pos_dict = calculate_positions(len(x_items), len(categories), box_width, None)
    axins = ax_left.inset_axes((0.38, 0.25, 0.45, 0.45))
    axins.set_facecolor('#f7f7f7')
    axins.spines['top'].set_visible(False)
    axins.spines['right'].set_visible(False)

    dp = [darken_color(c, factor=0.8) for c in palette_left]
    lp = [(r, g, b, 0.5) for r, g, b in palette_left]

    for i, cat in enumerate(categories):
        axins.boxplot(
            [plot_data_left[cat][g] for g in range(1, 3)],
            positions=[pos_dict[i][g] for g in range(1, 3)],
            widths=box_width, patch_artist=True, showfliers=outlier,
            boxprops=dict(facecolor=lp[i], edgecolor=dp[i], linewidth=1),
            whiskerprops={'linewidth': 0.6},
            medianprops={'color': dp[i], 'linewidth': 0.6},
            flierprops={'marker': 'o', 'color': 'grey', 'markersize': 2,
                        'markerfacecolor': lp[i], 'markeredgewidth': 0.1, 'markeredgecolor': dp[i]}
        )

    all_zoom_pos = [pos_dict[i][g] for g in range(1, 3) for i in range(len(categories))]
    axins.set_xlim(min(all_zoom_pos) - 0.35, max(all_zoom_pos) + 0.35)
    axins.set_ylim(-0.02, 0.34)
    axins.set_xticks([base_pos[g] for g in range(1, 3)])
    axins.set_xticklabels([x_items[g] for g in range(1, 3)],
                          fontsize=max(config.tick_label_fsize - 2, 6))
    axins.tick_params(axis='y', labelsize=config.tick_label_fsize)
    ax_left.indicate_inset_zoom(axins, edgecolor="grey", linewidth=0.5)

    # ==================================================
    # RIGHT PANELS: SP Runtime and LMP Runtime
    # ==================================================
    palette_right = sns.color_palette("gist_earth", n_colors=4)
    cats_right = ['30', '20', '10', '5']

    df_right = main_df.copy()
    df_right['subProIter'] = df_right['subProIter'].astype(str)
    df_right['epochLength'] = df_right['epochLength'].astype(str)
    x_items_r = np.sort(df_right['subProIter'].unique()).astype(str)

    for ax, val_col_r, ylabel_r, show_leg in [
        (ax_right1, 'SubProTime', 'SP Runtime (s)', False),
        (ax_right2, 'MPTimeAcc.', 'LMP Runtime (s)', True),
    ]:
        pd_r = prepare_boxplot_data(df_right, x_items_r, 'subProIter',
                                    cats_right, 'epochLength', val_col_r)
        plot_boxplot_group(
            ax=ax, plot_data=pd_r, categories=cats_right,
            palette=palette_right, config=config,
            show_outliers=outlier, show_ylabel=True,
            ylabel=ylabel_r, width=0.35,
            xlabel='CG Iterations', x_tick_labels=x_items_r,
            rotation=0, ylim=(0, 0.84),
            show_legend=show_leg,
            legend_loc='upper right',
            legend_title='Epoch size (s)' if show_leg else None,
            legend_ncol=2,
        )

    # ==================================================
    # Labels below each panel group + save
    # ==================================================
    fig.tight_layout(rect=[0, 0.1, 1, 1])

    left_pos = ax_left.get_position()
    r1_pos = ax_right1.get_position()
    r2_pos = ax_right2.get_position()
    label_y = min(left_pos.y0, r1_pos.y0) - 0.12

    fig.text((left_pos.x0 + left_pos.x1) / 2, label_y,
             'Relative Improvement', ha='center', va='top',
             fontsize=config.axis_label_fsize)
    fig.text((r1_pos.x0 + r2_pos.x1) / 2, label_y,
             'Pricing and RMP runtimes', ha='center', va='top',
             fontsize=config.axis_label_fsize)

    figure_path = os.path.join(os.path.dirname(data_path),
                               f'profile_combined_boxplot_{outlier}.pdf')
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path



def create_anytime_boxplot_double(data_path: str, config: PlotConfig) -> str:
    # Read main data
    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[(main_df['subProIter'] < 5)]
    main_df = main_df[main_df['Model'].isin(['CG', 'ISUD'])]

    legend_config = {
        'loc': 'upper right',
        'ncol': 2,
        'title': 'Epoch size (s)'
    }

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': main_df,
            'item_column': 'subProIter',
            'category_column': 'epochLength',
            'value_column': 'SubProTime',
            'categories': ['30', '20', '10', '5'],
            'category_labels': ['30', '20', '10', '5'],
            'ylabel': 'SP Runtime (s)',
            'xlabel': 'CG Iterations',
            'ylim': (0,0.84),
            'rotation': 0,
            'width': 0.35,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': main_df,
            'item_column': 'subProIter',
            'category_column': 'epochLength',
            'value_column': 'MPTimeAcc.',
            'categories': ['30', '20', '10', '5'],
            'category_labels': ['30', '20', '10', '5'],
            'ylabel': 'LMP Runtime (s)',
            'xlabel': 'CG Iterations',
            'ylim': (0,0.84),
            'rotation': 0,
            'width': 0.35,
            'show_outliers': outlier,
            'show_legend': True,  # shared legend only
            'legend_config': legend_config,
        }
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 4,
        'title': 'Epoch size (s)'
    }
    palette = sns.color_palette("gist_earth", n_colors=4)

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f"anytime_time_boxplot_{outlier}.pdf",
        fig_size=(8,4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
 #       tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
    )

def create_shuttle_wait_boxplot_three(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df

    Algorithm_Names = ['Iter_Fix_2_S1', 'Iter_Fix_2_S2','Iter_Fix_3_S1', 'Iter_Fix_3_S2', 'Greedy']
    Algorithm_Names_labels = ['Iter_Fix_2_S1', 'Iter_Fix_2_S2','Iter_Fix_3_S1', 'Iter_Fix_3_S2', 'Greedy']

    # Two subplot configurations: left = WaitTime, right = CommitWaitTime
    subplot_configs_1 = [
        {  # 2: Waiting Times
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance',
            'category_column': 'paramFile',
            'value_column': 'WaitTime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Waiting Times (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.35,
            'ylim': (0,14.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 2: Trip Delay
            #         'data_path': request_path,
            'data_df': data_df_request,
            'item_column': 'Instance',
            'category_column': 'paramFile',
            'value_column': 'TripDelay',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Trip Delay (min)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.35,
            'ylim': (0, 14.5),
            'show_outliers': outlier,
            'show_legend': False,  # we will use a shared legend
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance',
            'category_column': 'paramFile',
            'value_column': 'EpochRuntime',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': 'Instances',
            'ylim': (-0.02, 3.8),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.35,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance',
            'category_column': 'paramFile',
            'categories': Algorithm_Names,
            'category_labels': Algorithm_Names_labels,
            'value_column': 'nbRequests',
            'ylabel': '# Pending Requests',
            'xlabel': 'Instances',
            #          'ylim': (0,68),
            #          'ylim': (0, 305),
            'rotation': 15,
            'width': 0.35,
  #          'ylim': (0, 1050),
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
    ]

    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.75, 0.99),
        'ncol': 5,
        'title': 'Algorithm'
    }

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'shuttle_wait_boxplot_{outlier}.pdf',
        fig_size=(13,3),
        n_rows=1,
        n_cols=4,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.9],
    )

    return figure_path


def create_shuttle_time_boxplot_two(data_path: str, config: PlotConfig) -> str:
    # Read main data
    request_path = os.path.join(os.path.dirname(data_path), 'request_temp_data.csv')
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    palette = sns.color_palette("gist_earth", n_colors=3)

    if os.path.exists(request_path) and os.path.exists(time_path):
        processed_request_df = read_csv_with_encoding(request_path)
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)

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
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )
        processed_request_df['TripDelay'] = processed_request_df['TripDelay'] - 0.5

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'MP_Runtime', 'SubProbRuntime'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['paramFile', 'Instance_category', 'Algorithm_Name']
        )

        processed_request_df.to_csv(request_path, index=False)
        processed_time_df.to_csv(time_path, index=False)

    data_df_time = processed_time_df
    data_df_request = processed_request_df


    # Shared legend configuration (top center)
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 0.99),
        'ncol': 3,
        'title': 'Algorithm'
    }

    Alg_part1 = ['A-CG-CR-SP1', 'Greedy-CR']
    Alg_part1_labels = ['A-CG-C', 'Greedy']

    Alg_part2 = ['B-CG-CR']
    Alg_part2_labels = ['B-CG-C']


    subplot_configs_1 = [
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Alg_part1,
            'category_labels': Alg_part1_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': '        Demand Category',
            'ylim': (-0.02, 0.82),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.4,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
        },
        {  # 1: Runtime
            #          'data_path': request_path,
            'data_df': data_df_time,
            'item_column': 'Instance_category',
            'x_tick_labels': c.vehicle_groups_labels,
            'category_column': 'Algorithm_Name',
            'value_column': 'EpochRuntime',
            'categories': Alg_part2,
            'category_labels': Alg_part2_labels,
            'ylabel': 'Epoch Runtime (s)',
            'xlabel': '',
            'ylim': (-0.4, 31),
            #          'ylim': (0, 305),
            'rotation': 0,
            'width': 0.5,
            'target_lines': 30,
            'show_outliers': outlier,
            'show_legend': False,  # shared legend only
            'show_ylabel': False,
        },
    ]

    # Create ONE figure with two subplots
    figure_path = create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs_1,
        output_filename=f'process_two_time_boxplot_{outlier}.pdf',
        fig_size=(4.5,4),
        n_rows=1,
        n_cols=2,
        shared_legend=False,
        shared_legend_config=shared_legend_config,
        additional_filter=None,
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.86],
        palette=palette,
    )

    return figure_path


def create_anytime_time_request_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data



    main_df = read_csv_with_encoding(data_path)
    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, f"request_runtime_profiles")
    os.makedirs(folder_path, exist_ok=True)
    main_df["test"] = (main_df["Instance"] + "_E" + main_df["epochLength"].astype(str))

    Algorithm = ['MP_ISUD', 'A_CG']
    Labels = ['F-ICG', 'B-CG']

    processed_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', 'Epoch', 'ElapsedTime','waitTime'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['paramFile', 'Instance_category','epochLength','test']
    )

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * processed_df['epochLength']) / 60


    test_name = processed_df['test'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['test'] == test]

        palette = sns.color_palette("gist_earth", n_colors=4)
        palette = [palette[0], palette[2]]

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'EpochRuntime',
                "group_column": 'Algorithm',
                "groups": Algorithm,
                "group_labels": Labels,
                "ylabel": "Runtime (s)",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'nbRequests',
                "group_column": 'Algorithm',
                "groups": Algorithm,
                "group_labels": Labels,
                "ylabel": "# Pending requests",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                "linewidth": 0.8,
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
                "ncol": 2,
                "title": "Algorithm",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )

def create_anytime_dual_profile(data_path: str, config: PlotConfig) -> None:
    # Read main data
    time_path = os.path.join(os.path.dirname(data_path), 'time_temp_data.csv')

    if os.path.exists(time_path):
        processed_time_df = read_csv_with_encoding(time_path)
    else:
        main_df = read_csv_with_encoding(data_path)
        main_df["test"] = (main_df["Instance"] + "_E" + main_df["epochLength"].astype(str))

        processed_time_df = preprocess_nested_data(
            df=main_df,
            data_path=data_path,
            data_type='time',
            value_column=['nbRequests', 'EpochRuntime', '#passPerVehicle', '#requestPerVehicle', '#SP Iter',
                          'maxRoute', 'meanRoute', '#LGenerated', 'Epoch', 'ElapsedTime', 'maxDual', 'minDual',
                          'meanDual', 'meanDual'],
            instance_column='Instance',
            algorithm_column='Algorithm',
            aggregate_func=None,
            additional_columns=['Route_Recycle', 'paramFile', 'Instance_category','epochLength','test']
        )

        processed_time_df.to_csv(time_path, index=False)

    Algorithm = ['MP_ISUD', 'A_CG']
    Labels = ['F-ICG', 'B-CG']

    base_path = os.path.dirname(data_path)
    folder_path = os.path.join(base_path, f"dual_profiles")
    os.makedirs(folder_path, exist_ok=True)
    processed_df = processed_time_df

    processed_df['ElapsedTime'] = (processed_df['Epoch'] * processed_df['epochLength']) / 60
    test_name = processed_df['test'].unique()

    # Loop through tests
    for test in test_name:
        result_df = processed_df[processed_df['test'] == test]

        palette = sns.color_palette("gist_earth", n_colors=4)
        palette = [palette[0], palette[2]]

        # Ensure your results_df has columns:
        #   "x", "y_epoch", "y_nb_requests", "Setting"

        subplot_cfgs = [
            {
                "x_column": 'ElapsedTime',
                "y_column": 'maxDual',
                "group_column": 'Algorithm',
                "groups": Algorithm,
                "group_labels": Labels,
                "ylabel": "Max Dual",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": False,  # Top plot → hide x-label with shared x
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'meanDual',
                "group_column": 'Algorithm',
                "groups": Algorithm,
                "group_labels": Labels,
                "ylabel": "Mean Dual",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                "linewidth": 0.8,
            },
            {
                "x_column": 'ElapsedTime',
                "y_column": 'minDual',
                "group_column": 'Algorithm',
                "groups": Algorithm,
                "group_labels": Labels,
                "ylabel": "Min Dual",
                "xlabel": "Elapsed Time (min)",
                "show_xlabel": True,  # Bottom plot → x-label visible
                "show_ylabel": True,
                "xlim": (0, 240),
                "x_major_step": 30,
                "grid": True,
                "show_legend": False,
                "legend_config": {
                    "loc": "upper left",
                    "ncol": 1
                },
                "palette": palette,
                "linewidth": 0.8,
            }
        ]

        output_path = os.path.join(
            folder_path,
            f"{test}_dual_route.pdf"
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
                "ncol": 2,
                "title": "Algorithm",
            },
            tight_layout_rect=[0, 0, 1, 0.88],
        )

def create_anytime_violinplot(data_path: str, config: PlotConfig) -> str:
    """
    Plot MIP (integrality) gap as violin plot for algorithms A-CG-CR and B-CG-CR,
    grouped by Instance_category (vehicle supply).
    """
    import numpy as np

    main_df = read_csv_with_encoding(data_path)
    main_df = main_df[main_df['Algorithm'].isin(['A_CG'])]

    processed_time_df = preprocess_nested_data(
        df=main_df,
        data_path=data_path,
        data_type='time',
        value_column=['nbRequests', 'EpochRuntime', 'Objective', 'LinearObjective'],
        instance_column='Instance',
        algorithm_column='Algorithm',
        aggregate_func=None,
        additional_columns=['Ride_W2', 'Algorithm_Name', 'Instance_category','epochLength']
    )

    # MIP gap: 100 * (Objective - LinearObjective) / Objective (%)
    processed_time_df = processed_time_df.copy()
    processed_time_df['GAP'] = np.where(
        processed_time_df['Objective'] != 0,
        100.0 * (processed_time_df['Objective'] - processed_time_df['LinearObjective']) / processed_time_df['Objective'],
        np.nan,
    )
    processed_time_df = processed_time_df.dropna(subset=['GAP'])

    subplot_configs = [
        {
            'data_df': processed_time_df,
            'item_column': 'epochLength',
            'category_column': 'Algorithm',
            'value_column': 'GAP',
            'categories': ['A_CG'],
            'ylabel': 'Integrality Gap (%)',
            'xlabel': 'Instances',
            'rotation': 15,
            'width': 0.5,
  #          'ylim': (0, 0.17),
            'show_legend': False,
        },
    ]

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': (0.5, 1.02),
        'ncol': 1,
        'title': 'Algorithm',
    }

    figure_path = create_multi_subplot_violinplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs,
        output_filename='mip_gap_violinplot.pdf',
        fig_size=config.fig_size_small,
        n_rows=1,
        n_cols=1,
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_stats_legend=True,
        shared_stats_legend_config={
            'loc': 'upper right',
            'bbox': (0.98, 0.98),
            'ncol': 1,
        },
        color_reverse=False,
        tight_layout_rect=[0.0, 0.0, 1.0, 0.85],
    )
    return figure_path