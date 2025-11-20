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
    legend_labels = category_labels if category_labels else categories

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
                'color': 'grey',
                'markersize': 2,
                'markerfacecolor': light_palette[i],
                'markeredgewidth': 0.1
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
            legend_labels.append(f'Target ({target_lines})')
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
        ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)

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
        palette_name: str = "gist_earth"
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

    Returns:
    --------
    str : Path to saved figure
    """
    # Read data
    df = read_csv_with_encoding(data_path)

    # Apply global filter if provided
    if additional_filter is not None:
        df = additional_filter(df)

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
        fig_size = (6 * n_cols, 5 * n_rows)

    # Create figure and subplots
    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size)

    # Ensure axes is always a list
    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if n_rows > 1 or n_cols > 1 else [axes]

    # Get color palette
    max_categories = max(len(cfg['categories']) for cfg in subplot_configs)
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

        # Apply subplot-specific filter
        df_subplot = df.copy()
        if subplot_filter is not None:
            df_subplot = subplot_filter(df_subplot)

        # Ensure proper data types
        df_subplot[item_column] = df_subplot[item_column].astype(str)
        df_subplot[category_column] = df_subplot[category_column].astype(str)

        # Get sorted items
        x_items = np.sort(df_subplot[item_column].unique())

        # Prepare data
        plot_data = prepare_boxplot_data(
            df_subplot, x_items, item_column, categories,
            category_column, value_column
        )

        # Use subset of palette for this subplot
        subplot_palette = palette[:len(categories)]
#        cleaned_x_tick_labels = [item.split('.', 1)[1] for item in x_items]


        # Plot boxplots
        handles, labels = plot_boxplot_group(
            ax, plot_data, categories, subplot_palette, config,
            show_outliers=show_outliers,
            show_ylabel=True,
            ylabel=ylabel,
            width=width,
            gap_factors=gap_factors,
            target_lines=target_lines,
            xlabel=xlabel,
            x_tick_labels=x_items,
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
        target_line_label = shared_target_line_legend.get('label', f'Epoch Size ({target_line_value}s)')

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
    fig.savefig(figure_path, dpi=300, bbox_inches="tight")
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

    return create_multi_subplot_boxplots(
        data_path=data_path,
        config=config,
        subplot_configs=[subplot_config],
        output_filename=kwargs['output_filename'],
        fig_size=kwargs.get('fig_size', config.fig_size),
        additional_filter=kwargs.get('additional_filter', None),
        color_reverse=kwargs.get('color_reverse', True),
        palette_name=kwargs.get('palette_name', 'gist_earth')
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
                'label': f'Epoch Size ({target_lines}s)',
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
