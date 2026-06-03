import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import MultipleLocator
import seaborn as sns
import os
from typing import Dict, List, Optional, Tuple, Any, Union, Callable
from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.plot_config import PlotConfig

def prepare_boxplot_data(df, items, items_column, categories, category_column, value_column):
    """
    Prepare data for boxplot visualization.

    Parameters:
    -----------
    df : DataFrame
        The filtered dataframe
    items : array-like
        List of item values (e.g., instances, vehicle groups)
    items_column : str
        Column name for items
    categories : list
        List of category values to plot
    category_column : str
        Column name for categories
    value_column : str
        Column name for values to plot

    Returns:
    --------
    dict : Dictionary with categories as keys and lists of data as values
    """
    plot_data = {cat: [] for cat in categories}
    for item in items:
        for cat in categories:
            values = df[
                (df[items_column] == item) &
                (df[category_column].str.strip() == cat)
                ][value_column]
            plot_data[cat].append(values)
    return plot_data

def calculate_positions(n_xaxis_items, n_categories, width=0.5, gap_factors=None):
    """
    Calculate positions for boxplots based on number of categories.

    Parameters:
    -----------
    n_xaxis_items : int
        Number of x axis items
    n_categories : int
        Number of categories per instance
    width : float
        Width of each boxplot
    gap_factors : list, optional
        Additional gap factors for each category

    Returns:
    --------
    tuple : (base_positions, positions_dict)
    """
    base_positions = np.array(range(n_xaxis_items)) * 2
    positions = {}

    if gap_factors is None:
        gap_factors = [0] * n_categories

    # Calculate offset based on number of categories
    if n_categories == 1:
        offset = 0
    elif n_categories == 2:
        offset = width / 2
    elif n_categories == 3:
        offset = width
    elif n_categories == 4:
        offset = 3 * width / 2
    else:
        # General case for n categories
        offset = (n_categories - 1) * width / 2

    for i in range(n_categories):
        positions[i] = base_positions - offset + i * width + gap_factors[i]

    return base_positions, positions


def plot_boxplot_group(ax, plot_data, categories, palette, config: PlotConfig,
                       show_outliers=True, show_ylabel=True, ylabel='Value',
                       width=0.5, gap_factors=None, target_lines=None,
                       xlabel='', x_tick_labels=None, rotation=15,
                       ylim=None, show_legend=True, legend_loc='best',
                       legend_bbox=None, legend_title=None, category_labels=None,
                       show_grid=False, ylabel_pad=None, legend_ncol=1):
    """
    Create a boxplot on the given axis with enhanced flexibility.

    Parameters:
    -----------
    ax : matplotlib.axes.Axes
        The axis to plot on
    plot_data : dict
        Dictionary with categories as keys and lists of data as values
    categories : list
        List of category names
    palette : list
        Color palette for categories
    config : PlotConfig
        Configuration object with font sizes and styling
    show_outliers : bool
        Whether to show outliers
    show_ylabel : bool
        Whether to show y-axis label
    ylabel : str
        Y-axis label text
    width : float
        Width of boxplots
    gap_factors : list, optional
        Additional gap factors for positioning
    target_lines : dict or float, optional
        Reference lines to add. Can be a single value or dict of {label: value}
    xlabel : str
        X-axis label
    x_tick_labels : array-like
        Labels for x-axis ticks
    rotation : int
        Rotation angle for x-tick labels
    ylim : tuple or float
        Y-axis limits
    show_legend : bool
        Whether to show legend
    legend_loc : str or tuple
        Legend location
    legend_bbox : tuple
        Bbox anchor for legend
    legend_title : str
        Title for legend
    category_labels : list
        Custom labels for categories in legend
    show_grid : bool
        Whether to show grid
    ylabel_pad : float
        Padding for y-label
    """
    # Configure axes appearance
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    if show_grid:
        ax.grid(True, alpha=0.3, linestyle='--', axis='y')
        ax.set_axisbelow(True)

    # Prepare colors
    darkened_palette = [darken_color(color, factor=0.8) for color in palette]
    light_palette = [(r, g, b, 0.5) for r, g, b in palette]

    n_items = len(list(plot_data.values())[0])
    n_categories = len(categories)

    # Calculate positions
    base_positions, positions = calculate_positions(
        n_items, n_categories, width, gap_factors
    )

    legend_handles = []
    if category_labels:
        legend_labels = list(category_labels)  # make a copy
    else:
        legend_labels = list(categories)  # also copy here

    # Plot boxplots for each category
    for i, cat in enumerate(categories):
        pos = positions[i]
        bp = ax.boxplot(
            plot_data[cat],
            positions=pos,
            widths=width,
            patch_artist=True,
            showfliers=show_outliers,
            boxprops=dict(
                facecolor=light_palette[i],
                edgecolor=darkened_palette[i],
                linewidth=1
            ),
            whiskerprops={'linewidth': 0.6},
            medianprops={'color': darkened_palette[i], 'linewidth': 0.6},
            flierprops={
                'marker': 'o',
                'color': 'gray',
                'markersize': 1.2,
                'markerfacecolor': light_palette[i],
                'markeredgewidth': 0.1,
                'markeredgecolor': darkened_palette[i],
            }
        )
        legend_handles.append(bp["boxes"][0])

    # Add target lines if specified
    if target_lines is not None:
        if isinstance(target_lines, (int, float)):
            # Single target line
            line = ax.axhline(y=target_lines, color='red', linestyle='--',
                              linewidth=1.5, label=f'Target ({target_lines})')
            legend_handles.append(line)
            legend_labels.append(f'Epoch size ({target_lines}s)')
        elif isinstance(target_lines, dict):
            # Multiple target lines
            colors = ['red', 'blue', 'green', 'orange', 'purple']
            for idx, (label, value) in enumerate(target_lines.items()):
                color = colors[idx % len(colors)]
                line = ax.axhline(y=value, color=color, linestyle='--',
                                  linewidth=1.5, label=label)
                legend_handles.append(line)
                legend_labels.append(label)

    # Set labels
    if show_ylabel:
        if ylabel_pad:
            ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize,
                          fontweight='bold', labelpad=ylabel_pad)
        else:
            ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight='bold')

    if xlabel:
        ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize_small)

    # Set x-ticks
    if x_tick_labels is not None:
        ax.set_xticks(base_positions)
        ax.set_xticklabels(x_tick_labels, rotation=rotation, fontsize=config.tick_label_fsize)

    # Configure ticks
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.tick_params(which='minor', length=0)
    ax.tick_params(axis='both', which='major', labelsize=config.tick_label_fsize, length=3.8)

    # Set y-limits if specified
    if ylim is not None:
        if isinstance(ylim, (int, float)):
            ax.set_ylim(top=ylim)
        elif isinstance(ylim, tuple):
            ax.set_ylim(ylim)

    # Add legend if requested
    if show_legend and legend_handles:
        if legend_bbox:
            ax.legend(legend_handles, legend_labels,
                      loc=legend_loc, bbox_to_anchor=legend_bbox,
                      fontsize=config.legend_fsize,
                      edgecolor=config.legend_edgecolor,
                      title=legend_title,
                      ncol=legend_ncol,
                      title_fontproperties={'weight': 'bold',
                                            'size': config.legend_title_fsize})
        else:
            ax.legend(legend_handles, legend_labels,
                      loc=legend_loc,
                      fontsize=config.legend_fsize,
                      edgecolor=config.legend_edgecolor,
                      title=legend_title,
                      ncol=legend_ncol,
                      title_fontproperties={'weight': 'bold',
                                            'size': config.legend_title_fsize})

    return legend_handles, legend_labels


def create_multi_subplot_boxplots(
        data_path: str,
        config: PlotConfig,
        subplot_configs: List[Dict[str, Any]],
        output_filename: str,
        fig_size: Optional[Tuple[float, float]] = None,
        n_rows: Optional[int] = None,
        n_cols: Optional[int] = None,
        shared_legend: bool = False,
        shared_legend_config: Optional[Dict[str, Any]] = None,
        shared_target_line_legend: Optional[Dict[str, Any]] = None,
        figure_title: Optional[str] = None,
        tight_layout_rect: Optional[List[float]] = None,
        additional_filter: Optional[Callable] = None,
        color_reverse: bool = True,
        palette_name: str = "gist_earth",
        palette = None,
) -> str:
    """
    Create multiple boxplot subplots with maximum flexibility.

    Parameters:
    -----------
    data_path : str
        Path to the data file
    config : PlotConfig
        Plot configuration object
    subplot_configs : List[Dict]
        List of configurations for each subplot. Each dict should contain:
        - 'item_column': Column for x-axis items
        - 'category_column': Column for categories
        - 'value_column': Column for values to plot
        - 'categories': List of category values
        - 'category_labels': Labels for categories
        - 'ylabel': Y-axis label
        - 'xlabel': X-axis label
        - 'title': Subplot title (optional)
        - 'filter': Additional filter function for this subplot (optional)
        - 'ylim': Y-axis limits (optional)
        - 'target_lines': Reference lines (optional)
        - 'show_outliers': Whether to show outliers (default True)
        - 'width': Boxplot width (default 0.5)
        - 'rotation': X-label rotation (default 15)
        - 'show_legend': Whether to show legend for this subplot
        - 'legend_config': Dict with legend configuration
    output_filename : str
        Name of output file
    fig_size : tuple, optional
        Figure size (width, height)
    n_rows : int, optional
        Number of rows in subplot grid
    n_cols : int, optional
        Number of columns in subplot grid
    shared_legend : bool
        Whether to use a single shared legend for all subplots
    shared_legend_config : dict, optional
        Configuration for shared legend
    figure_title : str, optional
        Overall figure title
    tight_layout_rect : list, optional
        Rectangle for tight_layout
    additional_filter : callable, optional
        Global filter to apply to all data
    color_reverse : bool
        Whether to reverse color palette
    palette_name : str
        Name of color palette to use

    Each entry of `subplot_configs` may ALSO contain:

    - 'data_path': optional path to a CSV file for THIS subplot only.
                   If omitted, the global `data_path` is used.
    - 'data_df'  : optional pandas DataFrame for THIS subplot.
                   If provided, it overrides both global `data_path`
                   and subplot 'data_path' for that subplot.

    Returns:
    --------
    str : Path to saved figure
    """
    # Read data
    df_global = read_csv_with_encoding(data_path)
    if additional_filter is not None:
        df_global = additional_filter(df_global)

    # Cache to avoid re-reading the same CSV multiple times
    df_cache = {data_path: df_global}

    # Determine subplot layout
    n_subplots = len(subplot_configs)
    if n_rows is None and n_cols is None:
        n_cols = min(3, n_subplots)
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_rows is None:
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_cols is None:
        n_cols = (n_subplots + n_rows - 1) // n_rows

    # Determine figure size
    if fig_size is None:
        fig_size = (3 * n_cols, 4 * n_rows)

    # Create figure and subplots
    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size,
 #                            gridspec_kw={'width_ratios': [3, 2]}
                             )
 #   fig.subplots_adjust(wspace=0.1)
 #   axes[0].axhspan(0, 10, color="lightgray", alpha=0.4)
 #   axes[1].axhspan(0, 12, color="lightgray", alpha=0.4)

    # Ensure axes is always a list
    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if n_rows > 1 or n_cols > 1 else [axes]

    # Get color palette
    max_categories = max(len(cfg['categories']) for cfg in subplot_configs)
    if palette is None:
        palette = sns.color_palette(palette_name, n_colors=max_categories)
    if color_reverse:
        palette = palette[::-1]

    all_legend_handles = []
    all_legend_labels = []

    # Create each subplot
    for idx, subplot_config in enumerate(subplot_configs):
        ax = axes[idx]

        # Extract subplot-specific configuration
        item_column = subplot_config.get('item_column', 'Instance')
        x_tick_labels = subplot_config.get('x_tick_labels', None)
        category_column = subplot_config['category_column']
        value_column = subplot_config['value_column']
        categories = subplot_config['categories']
        category_labels = subplot_config.get('category_labels', categories)
        ylabel = subplot_config.get('ylabel', value_column)
        xlabel = subplot_config.get('xlabel', '')
        subplot_title = subplot_config.get('title', '')
        subplot_filter = subplot_config.get('filter', None)
        ylim = subplot_config.get('ylim', None)
        target_lines = subplot_config.get('target_lines', None)
        show_outliers = subplot_config.get('show_outliers', True)
        width = subplot_config.get('width', 0.5)
        rotation = subplot_config.get('rotation', 15)
        show_legend = subplot_config.get('show_legend', not shared_legend)
        legend_config = subplot_config.get('legend_config', {})
        gap_factors = subplot_config.get('gap_factors', None)
        show_grid = subplot_config.get('show_grid', False)
        show_ylabel = subplot_config.get('show_ylabel', True)

        # --- decide which dataframe this subplot uses ---
        subplot_data_path = subplot_config.get('data_path', None)
        subplot_df = subplot_config.get('data_df', None)

        if subplot_df is not None:
            # Directly provided DataFrame
            df_subplot = subplot_df.copy()
        else:
            if subplot_data_path is not None:
                # Subplot has its own CSV
                if subplot_data_path in df_cache:
                    df_subplot = df_cache[subplot_data_path].copy()
                else:
                    df_tmp = read_csv_with_encoding(subplot_data_path)
                    if additional_filter is not None:
                        df_tmp = additional_filter(df_tmp)
                    df_cache[subplot_data_path] = df_tmp
                    df_subplot = df_tmp.copy()
            else:
                # Fall back to global data
                df_subplot = df_global.copy()

            # Apply subplot-specific filter AFTER choosing the dataframe
        if subplot_filter is not None:
            df_subplot = subplot_filter(df_subplot)

        # Get sorted items
        x_items = np.sort(df_subplot[item_column].unique()).astype(str)

        # If category_column is None → create a dummy one
        if category_column is None:
            category_column = '_dummy_category'
            df_subplot[category_column] = str(subplot_config['categories'][0])
        else:
            df_subplot[category_column] = df_subplot[category_column].astype(str)

        df_subplot[item_column] = df_subplot[item_column].astype(str)
        df_subplot[category_column] = df_subplot[category_column].astype(str)


        # Prepare data
        plot_data = prepare_boxplot_data(
            df_subplot, x_items, item_column, categories,
            category_column, value_column
        )

        # Use subset of palette for this subplot
        subplot_palette = palette[:len(categories)]
  #      if len(categories) == 2:
  #          subplot_palette = [palette[0], palette[2]]
  #      if len(categories) == 1:
  #          subplot_palette = [palette[1]]

        if x_tick_labels is None:
            x_tick_labels = x_items
        # Plot boxplots
        handles, labels = plot_boxplot_group(
            ax, plot_data, categories, subplot_palette, config,
            show_outliers=show_outliers,
            show_ylabel=show_ylabel,
            ylabel=ylabel,
            width=width,
            gap_factors=gap_factors,
            target_lines=target_lines,
            xlabel=xlabel,
            x_tick_labels=x_tick_labels,
            rotation=rotation,
            ylim=ylim,
            show_legend=show_legend,
            legend_loc=legend_config.get('loc', 'best'),
            legend_bbox=legend_config.get('bbox', None),
            legend_title=legend_config.get('title', None),
            legend_ncol=legend_config.get('ncol', 1),
            category_labels=category_labels,
            show_grid=show_grid,
            ylabel_pad=subplot_config.get('ylabel_pad', None)
        )

        # Add subplot title if specified
        if subplot_title:
            ax.set_title(subplot_title, fontsize=config.title_fsize, fontweight='bold')

        # Collect handles for shared legend
        if shared_legend and idx == 0:
            all_legend_handles = handles
            all_legend_labels = labels

    # Hide unused subplots
    for idx in range(n_subplots, len(axes)):
        axes[idx].set_visible(False)

    # Add shared legend if requested
    if shared_legend and all_legend_handles:
        legend_config = shared_legend_config or {}
        # Filter out target line handles from main legend
        category_handles = [h for h in all_legend_handles if not isinstance(h, mlines.Line2D)]
        category_labels = all_legend_labels[:len(category_handles)]

        fig.legend(
            handles=category_handles,
            labels=category_labels,
            loc=legend_config.get('loc', 'upper center'),
            bbox_to_anchor=legend_config.get('bbox', (0.5, 0.98)),
            ncol=legend_config.get('ncol', len(category_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_config.get('title', None),
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
            framealpha=legend_config.get('framealpha', 1.0),
            facecolor=legend_config.get('facecolor', 'white')
        )

    # Add separate target line legend if specified
    if shared_target_line_legend:
        target_line_value = shared_target_line_legend.get('value', 30)
        target_line_label = shared_target_line_legend.get('label', f'Epoch size ({target_line_value}s)')

        fig.legend(
            handles=[mlines.Line2D([], [], color='red', linestyle='--', lw=1.5)],
            labels=[target_line_label],
            loc=shared_target_line_legend.get('loc', 'upper center'),
            bbox_to_anchor=shared_target_line_legend.get('bbox', (0.5, 0.92)),
            ncol=1,
            fontsize=config.legend_fsize,
            labelspacing=0.45,
            edgecolor="none"
        )

    # Add figure title if specified
    if figure_title:
        fig.suptitle(figure_title, fontsize=config.title_fsize + 2,
                     fontweight='bold', y=1.02)

    # Adjust layout
    if tight_layout_rect:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()

    # Save figure
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path)
    plt.close(fig)

    return figure_path


# Convenience functions for common use cases
def create_single_boxplot(data_path, config, **kwargs):
    """Convenience function for single boxplot."""
    subplot_config = {
        'item_column': kwargs.get('item_column', 'Instance'),
        'category_column': kwargs['category_column'],
        'value_column': kwargs['value_column'],
        'categories': kwargs['categories'],
        'category_labels': kwargs.get('category_labels', kwargs['categories']),
        'ylabel': kwargs.get('ylabel', 'Value'),
        'xlabel': kwargs.get('xlabel', 'Instances'),
        'ylim': kwargs.get('ylim', None),
        'target_lines': kwargs.get('target_lines', None),
        'show_outliers': kwargs.get('show_outliers', True),
        'width': kwargs.get('width', 0.5),
        'rotation': kwargs.get('rotation', 15),
        'gap_factors': kwargs.get('gap_factors', None),
        'show_legend': True,
        'legend_config': {
            'loc': kwargs.get('legend_loc', 'best'),
            'bbox': kwargs.get('legend_bbox', None),
            'title': kwargs.get('legend_title', None),
            'ncol': kwargs.get('legend_ncol', 1),
        }
    }
    tight_layout_rect = kwargs.get('tight_layout_rect', None)

    return create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=[subplot_config],
        output_filename=kwargs['output_filename'],
        fig_size=kwargs.get('fig_size', config.fig_size),
        additional_filter=kwargs.get('additional_filter', None),
        color_reverse=kwargs.get('color_reverse', True),
        palette_name=kwargs.get('palette_name', 'gist_earth'),
        tight_layout_rect=tight_layout_rect,
    )


def create_comparison_boxplots(data_path, config, comparison_column, comparison_values, **kwargs):
    """Convenience function for side-by-side comparison boxplots with support for dual legends."""
    subplot_configs = []

    # Check if we have target lines
    target_lines = kwargs.get('target_lines', None)

    for i, value in enumerate(comparison_values):
        subplot_config = {
            'item_column': kwargs.get('item_column', 'Instance'),
            'category_column': kwargs['category_column'],
            'value_column': kwargs['value_column'],
            'categories': kwargs['categories'],
            'category_labels': kwargs.get('category_labels', kwargs['categories']),
            'ylabel': kwargs.get('ylabel', 'Value') if i == 0 else '',
            'xlabel': f"$W_{{delay}}$ = {value}",
            'filter': lambda df, val=value: df[df[comparison_column] == val],
            'ylim': kwargs.get('ylim', None),
            'target_lines': target_lines,
            'show_outliers': kwargs.get('show_outliers', True),
            'width': kwargs.get('width', 0.5),
            'rotation': kwargs.get('rotation', 15),
            'gap_factors': kwargs.get('gap_factors', None),
            'show_legend': False,
        }
        subplot_configs.append(subplot_config)

    # Prepare shared legend config
    shared_legend_config = {
        'loc': 'upper center',
        'bbox': kwargs.get('legend_bbox', (0.5, 0.98)),
        'ncol': kwargs.get('legend_ncol', 1),
        'title': kwargs.get('legend_title', None)
    }
    tight_layout_rect =  kwargs.get('tight_layout_rect', None)

    # Prepare target line legend if target lines exist
    shared_target_line_legend = None
    if target_lines is not None:
        if isinstance(target_lines, (int, float)):
            shared_target_line_legend = {
                'value': target_lines,
                'label': f'Epoch size ({target_lines}s)',
                'bbox': kwargs.get('legend_bbox2', (0.5, 0.92))
            }
        elif isinstance(target_lines, dict):
            # If target_lines is a dict, take the first entry for the shared legend
            first_label = list(target_lines.keys())[0]
            first_value = target_lines[first_label]
            shared_target_line_legend = {
                'value': first_value,
                'label': first_label,
                'bbox': kwargs.get('legend_bbox2', (0.5, 0.92))
            }

    return create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=subplot_configs,
        output_filename=kwargs['output_filename'],
        fig_size=kwargs.get('fig_size', None),
        n_rows=1,
        n_cols=len(comparison_values),
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        shared_target_line_legend=shared_target_line_legend,
        additional_filter=kwargs.get('additional_filter', None),
        color_reverse=kwargs.get('color_reverse', True),
        tight_layout_rect = tight_layout_rect,
    )


def plot_epoch_vehicle_boxplot(
        results_df,
        output_path: str,
        config: PlotConfig,
        ylim: float,
        value_column: str = "nbNodes",
        ylabel: str = "nbNodes",
):
    """
    Create an epoch boxplot (nbNodes vs Epoch) with:
    - results_df instead of data_path
    - saving to output_path
    - epochs 1..240 (all epochs)
    - sparse tick labels only at selected positions
    - no legend
    """

    # Make sure epochs are integers and sorted
    df_plot = results_df.copy()
    df_plot["Epoch"] = df_plot["Epoch"].astype(int)

    # Use all epochs from 1 to 240
    kept_epochs = np.arange(1, 241)

    # Build list of lists: boxplot_data[i] contains values for epoch_i
    boxplot_data = []
    for e in kept_epochs:
        vals = df_plot.loc[df_plot["Epoch"] == e, value_column]
        if len(vals) == 0:
            # keep structure even if some epochs have no data
            boxplot_data.append([np.nan])
        else:
            boxplot_data.append(vals.values)

    # Create figure
    fig, ax = plt.subplots(figsize=config.fig_size_wide)

    # Plot boxplots at their actual epoch numbers (1..240)
    bp = ax.boxplot(
        boxplot_data,
        positions=kept_epochs,
        widths=0.2,
        showfliers=True,
        patch_artist=True,
        boxprops=dict(facecolor=(0.6, 0.8, 0.6, 0.5),
                      edgecolor="green", linewidth=1),
        medianprops=dict(color="green", linewidth=1),
        whiskerprops=dict(color="black", linewidth=0.7),
        flierprops=dict(marker='o', markersize=2, markerfacecolor="green",
                        markeredgecolor="black", linestyle="none"),
    )

    # Axis labels
    ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")
    ax.set_xlabel("Runtime Epochs", fontsize=config.axis_label_fsize_small,
                  fontweight="bold")

    # X-ticks: only display at key positions (you can change these lists)
    tick_positions = [0, 30, 60, 90, 120, 150, 180, 210, 240]
    tick_labels    = [0, 30, 60, 90, 120, 150, 180, 210, 240]

    ax.set_xticks(tick_positions)
    ax.set_xticklabels(tick_labels, fontsize=config.tick_label_fsize, rotation=0)

    # Ensure the 0 tick is visible
    ax.set_xlim(left=0, right=max(kept_epochs) + 1)
    ax.set_ylim(0, ylim)

    # Clean axes
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(axis='both', which='major', length=3.8)

    # No legend

    plt.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def create_anytime_improve_boxplot_with_zoom1(
    data_path: str,
    config: PlotConfig,
    outlier: bool = True,
    output_filename: str = None,
    inset_group_idx: int = 1,              # 0-based -> 1 means second column
    inset_bounds=(0.30, 0.18, 0.28, 0.28), # (x0, y0, w, h) in axes fraction
    inset_ylim=(-0.01, 0.22),
    inset_xpad=0.18,
    additional_filter: Optional[Callable] = None,
) -> str:
    """
    Create the anytime improvement boxplot with an inset zoom on one x-group.

    This is a standalone function and does not require changing any of your
    existing plotting functions.
    """

    def preprocess_anytime(df):
        return df[df['subProIter'] < 5].copy()



    # ----------------------------
    # Load and preprocess data
    # ----------------------------
    df = read_csv_with_encoding(data_path)

    if additional_filter is not None:
        df = additional_filter(df)

    item_column = 'subProIter'
    category_column = 'epochLength'
    value_column = 'RelativeImprove'
    categories = ['30', '20', '10', '5']

    df[item_column] = df[item_column].astype(str)
    df[category_column] = df[category_column].astype(str)

    x_items = np.sort(df[item_column].unique()).astype(str)

    plot_data = prepare_boxplot_data(
        df, x_items, item_column, categories, category_column, value_column
    )

    # ----------------------------
    # Figure + main plot
    # ----------------------------
    fig, ax = plt.subplots(figsize=config.fig_size)

    palette = sns.color_palette("gist_earth", n_colors=len(categories))

    plot_boxplot_group(
        ax=ax,
        plot_data=plot_data,
        categories=categories,
        palette=palette,
        config=config,
        show_outliers=outlier,
        show_ylabel=True,
        ylabel='Relative Improve %',
        width=0.4,
        gap_factors=None,
        target_lines=None,
        xlabel='Iterations',
        x_tick_labels=x_items,
        rotation=0,
        ylim=(-0.3, 22),
        show_legend=True,
        legend_loc='upper right',
        legend_bbox=None,
        legend_title='Epoch size (s)',
        category_labels=None,
        show_grid=False,
        ylabel_pad=None,
        legend_ncol=2
    )

    # ----------------------------
    # Build inset zoom
    # ----------------------------
    n_items = len(x_items)
    n_categories = len(categories)

    base_positions, positions = calculate_positions(
        n_items,
        n_categories,
        0.4,
        None
    )

    # Create inset
    axins = ax.inset_axes(inset_bounds)

    # Match basic style
    axins.spines['top'].set_visible(False)
    axins.spines['right'].set_visible(False)

    # Reuse the same color prep logic as plot_boxplot_group
    darkened_palette = [darken_color(color, factor=0.8) for color in palette]
    light_palette = [(r, g, b, 0.5) for r, g, b in palette]

    # Draw only the selected group in the inset
    for i, cat in enumerate(categories):
        pos = [positions[i][inset_group_idx]]
        data_for_group = [plot_data[cat][inset_group_idx]]

        axins.boxplot(
            data_for_group,
            positions=pos,
            widths=0.4,
            patch_artist=True,
            showfliers=outlier,
            boxprops=dict(
                facecolor=light_palette[i],
                edgecolor=darkened_palette[i],
                linewidth=1
            ),
            whiskerprops={'linewidth': 0.6},
            medianprops={'color': darkened_palette[i], 'linewidth': 0.6},
            flierprops={
                'marker': 'o',
                'color': 'grey',
                'markersize': 2,
                'markerfacecolor': light_palette[i],
                'markeredgewidth': 0.1
            }
        )

    axins.set_facecolor('#f7f7f7')

    # Zoom around the second column
    group_positions = [positions[i][inset_group_idx] for i in range(n_categories)]
    axins.set_xlim(min(group_positions) - inset_xpad, max(group_positions) + inset_xpad)
    axins.set_ylim(-0.02, 0.32)

    # Optional inset ticks
    axins.set_xticks([base_positions[inset_group_idx]])
    axins.set_xticklabels([x_items[inset_group_idx]], fontsize=max(config.tick_label_fsize - 2, 6))
    axins.tick_params(axis='y', which='major', labelsize=config.tick_label_fsize)

    # Draw rectangle + connectors
    ax.indicate_inset_zoom(axins, edgecolor="grey", linewidth=0.5)

    # ----------------------------
    # Layout + save
    # ----------------------------
    fig.tight_layout()

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path


def create_anytime_improve_boxplot_with_zoom(
    data_path: str,
    config: PlotConfig,
    outlier: bool = True,
    output_filename: str = None,
    item_column: str = 'subProIter',
    category_column: str = 'epochLength',
    value_column: str = 'RelativeImprove',
    ylabel: str = 'RelativeImprove',
    xlabel: str = 'CG Iterations',
    ylim=(-0.3, 22),
    categories: Optional[List[str]] = None,
    box_width: float = 0.4,
    inset_group_idx: int = 1,                # start group: 0-based -> 1 means second column
    inset_group_end_idx: int = 2,            # end group: 0-based -> 2 means third column
    inset_bounds=(0.30, 0.18, 0.28, 0.28),   # (x0, y0, w, h) in axes fraction
    inset_ylim=(-0.02, 0.32),
    inset_xpad=0.18,
    additional_filter: Optional[Callable] = None,
) -> str:
    """
    Create the anytime improvement boxplot with an inset zoom on a range of x-groups.

    By default, the inset zooms into the second and third columns.
    """
    if categories is None:
        categories = ['30', '20', '10', '5']

    # ----------------------------
    # Load and preprocess data
    # ----------------------------
    df = read_csv_with_encoding(data_path)

    if additional_filter is not None:
        df = additional_filter(df)

    df[item_column] = df[item_column].astype(str)
    df[category_column] = df[category_column].astype(str)

    x_items = np.sort(df[item_column].unique()).astype(str)

    plot_data = prepare_boxplot_data(
        df, x_items, item_column, categories, category_column, value_column
    )

    # ----------------------------
    # Figure + main plot
    # ----------------------------
    fig, ax = plt.subplots(figsize=(4, 4))

    palette = sns.color_palette("gist_earth", n_colors=len(categories))

    plot_boxplot_group(
        ax=ax,
        plot_data=plot_data,
        categories=categories,
        palette=palette,
        config=config,
        show_outliers=outlier,
        show_ylabel=True,
        ylabel=ylabel,
        width=box_width,
        gap_factors=None,
        target_lines=None,
        xlabel=xlabel,
        x_tick_labels=x_items,
        rotation=0,
        ylim=ylim,
        show_legend=True,
        legend_loc='upper right',
        legend_bbox=None,
        legend_title='Epoch size (s)',
        category_labels=None,
        show_grid=False,
        ylabel_pad=None,
        legend_ncol=2
    )

    # ----------------------------
    # Build inset zoom
    # ----------------------------
    n_items = len(x_items)
    n_categories = len(categories)

    base_positions, positions = calculate_positions(
        n_items,
        n_categories,
        box_width,
        None
    )

    # Keep indices valid
    start_idx = max(0, inset_group_idx)
    end_idx = min(inset_group_end_idx, n_items - 1)

    # Create inset
    axins = ax.inset_axes(inset_bounds)
    axins.set_facecolor('#f7f7f7')

    # Match basic style
    axins.spines['top'].set_visible(False)
    axins.spines['right'].set_visible(False)

    # Reuse the same color prep logic as plot_boxplot_group
    darkened_palette = [darken_color(color, factor=0.8) for color in palette]
    light_palette = [(r, g, b, 0.5) for r, g, b in palette]

    # Draw the selected group range in the inset
    for i, cat in enumerate(categories):
        pos = [positions[i][g] for g in range(start_idx, end_idx + 1)]
        data_for_groups = [plot_data[cat][g] for g in range(start_idx, end_idx + 1)]

        axins.boxplot(
            data_for_groups,
            positions=pos,
            widths=box_width,
            patch_artist=True,
            showfliers=outlier,
            boxprops=dict(
                facecolor=light_palette[i],
                edgecolor=darkened_palette[i],
                linewidth=1
            ),
            whiskerprops={'linewidth': 0.6},
            medianprops={'color': darkened_palette[i], 'linewidth': 0.6},
            flierprops={
                'marker': 'o',
                'color': 'grey',
                'markersize': 2,
                'markerfacecolor': light_palette[i],
                'markeredgewidth': 0.1
            }
        )

    # Zoom around the selected columns
    all_group_positions = []
    for g in range(start_idx, end_idx + 1):
        all_group_positions.extend([positions[i][g] for i in range(n_categories)])

    axins.set_xlim(min(all_group_positions) - inset_xpad,
                   max(all_group_positions) + inset_xpad)
    axins.set_ylim(inset_ylim)

    # Optional inset ticks
    tick_positions = [base_positions[g] for g in range(start_idx, end_idx + 1)]
    tick_labels = [x_items[g] for g in range(start_idx, end_idx + 1)]
    axins.set_xticks(tick_positions)
    axins.set_xticklabels(
        tick_labels,
        fontsize=max(config.tick_label_fsize - 2, 6)
    )
    axins.tick_params(axis='y', which='major', labelsize=config.tick_label_fsize)

    # Draw rectangle + connectors
    ax.indicate_inset_zoom(axins, edgecolor="grey", linewidth=0.5)

    # ----------------------------
    # Layout + save
    # ----------------------------
    fig.tight_layout()

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path

def create_anytime_improve_lineplot_with_zoom(
    data_path: str,
    config: PlotConfig,
    outlier: bool = True,  # unused, kept for compatibility
    output_filename: str = None,
    item_column: str = 'subProIter',
    category_column: str = 'epochLength',
    value_column: str = 'RelativeImprove',
    ylabel: str = 'RelativeImprove',
    xlabel: str = 'CG Iterations',
    ylim=(-0.3, 22),
    categories: Optional[List[str]] = None,
    box_width: float = 0.4,  # unused, kept for compatibility
    inset_group_idx: int = 1,
    inset_group_end_idx: int = 2,
    inset_bounds=(0.30, 0.18, 0.28, 0.28),
    inset_ylim=(-0.02, 0.32),
    inset_xpad=0.18,
    additional_filter: Optional[Callable] = None,
) -> str:
    """
    Create an anytime improvement line plot showing only averages,
    with an inset zoom on a selected range of x-groups.
    """
    if categories is None:
        categories = ['30', '20', '10', '5']

    # ----------------------------
    # Load and preprocess data
    # ----------------------------
    df = read_csv_with_encoding(data_path)

    if additional_filter is not None:
        df = additional_filter(df)

    df[item_column] = df[item_column].astype(str)
    df[category_column] = df[category_column].astype(str)

    # robust numeric-ish sort for x labels
    def _sort_key(v):
        try:
            return float(v)
        except ValueError:
            return v

    x_items = sorted(df[item_column].unique(), key=_sort_key)
    x_pos = np.arange(len(x_items))

    # Compute mean values for each category across x_items
    mean_data = {}
    for cat in categories:
        cat_means = []
        for x in x_items:
            vals = df.loc[
                (df[item_column] == x) & (df[category_column] == cat),
                value_column
            ].dropna()

            cat_means.append(vals.mean() if len(vals) > 0 else np.nan)

        mean_data[cat] = cat_means

    # ----------------------------
    # Figure + main plot
    # ----------------------------
    fig, ax = plt.subplots(figsize=(3.5, 4))
    palette = sns.color_palette("gist_earth", n_colors=len(categories))

    for i, cat in enumerate(categories):
        ax.plot(
            x_pos,
            mean_data[cat],
            marker='o',
            linewidth=1.5,
            markersize=4,
            color=palette[i],
            label=cat
        )

    ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.set_ylabel(ylabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_items, rotation=0, fontsize=config.tick_label_fsize)
    ax.tick_params(axis='y', which='major', labelsize=config.tick_label_fsize)
    ax.set_ylim(ylim)

    ax.legend(
        title='Epoch size (s)',
        loc='upper right',
        ncol=2,
        fontsize=config.legend_fsize,
        title_fontsize=config.legend_fsize
    )

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(False)

    # ----------------------------
    # Build inset zoom
    # ----------------------------
    n_items = len(x_items)
    start_idx = max(0, inset_group_idx)
    end_idx = min(inset_group_end_idx, n_items - 1)

    axins = ax.inset_axes(inset_bounds)
    axins.set_facecolor('#f7f7f7')
    axins.spines['top'].set_visible(False)
    axins.spines['right'].set_visible(False)

    zoom_x = x_pos[start_idx:end_idx + 1]

    for i, cat in enumerate(categories):
        zoom_y = mean_data[cat][start_idx:end_idx + 1]
        axins.plot(
            zoom_x,
            zoom_y,
            marker='o',
            linewidth=1.2,
            markersize=3,
            color=palette[i]
        )

    axins.set_xlim(
        x_pos[start_idx] - inset_xpad,
        x_pos[end_idx] + inset_xpad
    )
    axins.set_ylim(inset_ylim)

    axins.set_xticks(zoom_x)
    axins.set_xticklabels(
        x_items[start_idx:end_idx + 1],
        fontsize=max(config.tick_label_fsize - 2, 6)
    )
    axins.tick_params(axis='y', which='major', labelsize=max(config.tick_label_fsize - 2, 6))

    ax.indicate_inset_zoom(axins, edgecolor="grey", linewidth=0.5)

    # ----------------------------
    # Layout + save
    # ----------------------------
    fig.tight_layout()

    if output_filename is None:
        output_filename = "anytime_lineplot_with_zoom.pdf"

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path

def create_anytime_epoch_count_barplot(
    data_path: str,
    config: PlotConfig,
    output_filename: str = None,
    item_column: str = 'subProIter',
    category_column: str = 'epochLength',
    ylabel: str = '# Epochs',
    xlabel: str = 'CG Iterations',
    categories: Optional[List[str]] = None,
    bar_width: float = 0.18,
    additional_filter: Optional[Callable] = None,
) -> str:
    """
    Grouped bar plot:
      - x-axis: item_column (e.g. subProIter)
      - y-axis: number of rows
      - one bar per category_column value (e.g. epochLength)
    """
    if categories is None:
        categories = ['30', '20', '10', '5']

    # ----------------------------
    # Load and preprocess data
    # ----------------------------
    df = read_csv_with_encoding(data_path)

    if additional_filter is not None:
        df = additional_filter(df)

    df[item_column] = df[item_column].astype(str)
    df[category_column] = df[category_column].astype(str)

    def _sort_key(v):
        try:
            return float(v)
        except ValueError:
            return v

    x_items = sorted(df[item_column].unique(), key=_sort_key)
    x = np.arange(len(x_items))

    # ----------------------------
    # Count rows for each (item, category)
    # ----------------------------
    num_epochs = [1932,2901, 5775, 11550]
    epochs_by_item = dict(zip(categories, num_epochs))
    count_data = {}
    for cat in categories:
        counts = []
        for item in x_items:
            n = len(df[(df[item_column] == item) & (df[category_column] == cat)])
            counts.append(n/epochs_by_item[cat])
        count_data[cat] = counts

    # ----------------------------
    # Plot
    # ----------------------------
    fig, ax = plt.subplots(figsize=(3.5, 4))
    palette = sns.color_palette("gist_earth", n_colors=len(categories))

    n_categories = len(categories)
    offsets = (np.arange(n_categories) - (n_categories - 1) / 2) * bar_width

    for i, cat in enumerate(categories):
        ax.bar(
            x + offsets[i],
            count_data[cat],
            width=bar_width,
            color=palette[i],
            label=cat
        )

    ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.set_ylabel(ylabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.set_xticks(x)
    ax.set_xticklabels(x_items, rotation=0, fontsize=config.tick_label_fsize)
    ax.tick_params(axis='y', which='major', labelsize=config.tick_label_fsize)

    ax.legend(
        title='Epoch size (s)',
        loc='upper right',
        ncol=2,
        fontsize=config.legend_fsize,
        title_fontsize=config.legend_fsize
    )

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(False)

    fig.tight_layout()

    if output_filename is None:
        output_filename = "anytime_epoch_count_barplot.pdf"

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)

    return figure_path