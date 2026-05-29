import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import MultipleLocator
import seaborn as sns
import os
from typing import Dict, List, Optional, Tuple, Any, Union, Callable
from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.plot_config import PlotConfig

# ============================================================
# Line plot utilities
# ============================================================

def prepare_lineplot_data(
    df,
    items,
    items_column: str,
    categories: List[Any],
    category_column: str,
    value_column: str,
    agg_func: Callable = np.mean
) -> Dict[Any, List[float]]:
    """
    Prepare aggregated data for line plot visualization.

    For each (item, category) pair, the values in `value_column` are aggregated
    using `agg_func` (e.g., np.mean, np.median). If no values are found, np.nan
    is appended so the line can still be plotted.

    Parameters
    ----------
    df : DataFrame
        Filtered dataframe.
    items : array-like
        List of x-axis item values.
    items_column : str
        Column name for items.
    categories : list
        List of category values to plot (one line per category).
    category_column : str
        Column name for categories.
    value_column : str
        Column containing numeric values to aggregate.
    agg_func : callable
        Aggregation function, default is np.mean.

    Returns
    -------
    dict
        Dictionary with categories as keys and list of aggregated values as values.
    """
    line_data = {cat: [] for cat in categories}

    for item in items:
        mask_item = df[items_column] == item
        for cat in categories:
            mask_cat = df[category_column].str.strip() == str(cat).strip()
            values = df.loc[mask_item & mask_cat, value_column]

            if len(values) == 0:
                line_data[cat].append(np.nan)
            else:
                line_data[cat].append(agg_func(values.values))

    return line_data


def plot_line_group(
    ax,
    line_data: Dict[Any, List[float]],
    categories: List[Any],
    palette: List[Tuple[float, float, float]],
    config: PlotConfig,
    show_ylabel: bool = True,
    ylabel: str = 'Value',
    xlabel: str = '',
    x_tick_labels: Optional[List[Any]] = None,
    rotation: int = 15,
    ylim: Optional[Union[Tuple[float, float], float]] = None,
    show_legend: bool = True,
    legend_loc: Union[str, Tuple[float, float]] = 'best',
    legend_bbox: Optional[Tuple[float, float]] = None,
    legend_title: Optional[str] = None,
    category_labels: Optional[List[str]] = None,
    show_grid: bool = False,
    ylabel_pad: Optional[float] = None,
    line_styles: Optional[List[str]] = None,
    marker_styles: Optional[List[str]] = None,
    line_width: float = 1.5,
    marker_size: float = 4.0,
    target_lines: Optional[Union[float, Dict[str, float]]] = None,
    show_y_ticks: bool = True,        # <-- NEW
):

    """
    Plot multiple line series (one per category) on a single axis.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Axis to plot on.
    line_data : dict
        Dictionary with categories as keys and list of y-values as values.
    categories : list
        Category names (one line per category).
    palette : list
        List of colors for each category.
    config : PlotConfig
        Plot configuration object.
    show_ylabel : bool
        Whether to display y-axis label.
    ylabel : str
        Y-axis label.
    xlabel : str
        X-axis label.
    x_tick_labels : list, optional
        Labels for x-axis ticks.
    rotation : int
        Rotation of x-axis tick labels.
    ylim : tuple or float, optional
        Y-axis limits.
    show_legend : bool
        Whether to show legend.
    legend_loc : str or tuple
        Legend location.
    legend_bbox : tuple, optional
        Bbox anchor for legend.
    legend_title : str, optional
        Legend title.
    category_labels : list, optional
        Custom labels for legend (same length as categories).
    show_grid : bool
        Whether to show grid.
    ylabel_pad : float, optional
        Padding for y-label.
    line_styles : list of str, optional
        Custom line styles per category (e.g., ['-', '--']).
    marker_styles : list of str, optional
        Custom marker styles per category (e.g., ['o', 's']).
    line_width : float
        Line width.
    marker_size : float
        Marker size.
    target_lines : float or dict, optional
        Horizontal reference lines (same behaviour as in boxplots).

    Returns
    -------
    legend_handles, legend_labels
        For use with shared legends in multi-subplot figures.
    """
    # Axes appearance
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    if show_grid:
        ax.grid(True, alpha=0.3, linestyle='--', axis='y')
        ax.set_axisbelow(True)

    n_points = len(next(iter(line_data.values())))
    x = np.arange(n_points)

    legend_handles = []
    legend_labels = category_labels if category_labels else [str(c) for c in categories]

    # Default styles if not provided
    if line_styles is None:
        line_styles = ['-'] * len(categories)
    if marker_styles is None:
        marker_styles = ['o'] * len(categories)

    # Darken palette a bit for better contrast
    darkened_palette = [darken_color(color, factor=0.8) for color in palette]
    light_palette = [(r, g, b, 0.6) for (r, g, b) in palette]

    # Plot lines
    for i, cat in enumerate(categories):
        y = line_data[cat]

        style = line_styles[i % len(line_styles)]
        marker = marker_styles[i % len(marker_styles)]
        color = palette[i % len(palette)]
        face_color = palette[i % len(light_palette)]

        handle, = ax.plot(
            x, y,
            linestyle=style,
            marker=marker,
            linewidth=line_width,
            markersize=marker_size,
            color=color,
            markerfacecolor=face_color,  # light face
            markeredgecolor=color,  # line color edge
            markeredgewidth=1.2,
            label=str(cat)
        )
        legend_handles.append(handle)

    # Target lines
    if target_lines is not None:
        if isinstance(target_lines, (int, float)):
            line = ax.axhline(
                y=target_lines,
                color='red',
                linestyle='--',
                linewidth=1.5,
                label=f'Target ({target_lines})'
            )
            legend_handles.append(line)
            legend_labels.append(f'Target ({target_lines})')
        elif isinstance(target_lines, dict):
            colors = ['red', 'blue', 'green', 'orange', 'purple']
            for idx, (label, value) in enumerate(target_lines.items()):
                color = colors[idx % len(colors)]
                line = ax.axhline(
                    y=value,
                    color=color,
                    linestyle='--',
                    linewidth=1.5,
                    label=label
                )
                legend_handles.append(line)
                legend_labels.append(label)

    # Labels
    if show_ylabel:
        if ylabel_pad:
            ax.set_ylabel(
                ylabel,
                fontsize=config.axis_label_fsize,
                fontweight='bold',
                labelpad=ylabel_pad
            )
        else:
            ax.set_ylabel(
                ylabel,
                fontsize=config.axis_label_fsize,
                fontweight='bold'
            )

    if xlabel:
        ax.set_xlabel(
            xlabel,
            fontsize=config.axis_label_fsize,
            fontweight='bold'
        )

    # X ticks
    if x_tick_labels is not None:
        ax.set_xticks(x)
        ax.set_xticklabels(
            x_tick_labels,
            rotation=rotation,
            fontsize=config.tick_label_fsize
        )

    # Configure ticks
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.tick_params(which='minor', length=0)
    ax.tick_params(axis='both', which='major',
                   labelsize=config.tick_label_fsize, length=3.8)

    # Hide y-axis ticks/labels/spine if requested
    if not show_y_ticks:
        ax.set_yticklabels([])
        ax.tick_params(axis='y', which='both', length=0)
        ax.spines['left'].set_visible(False)

    # Y limits
    if ylim is not None:
        if isinstance(ylim, (int, float)):
            ax.set_ylim(top=ylim)
        elif isinstance(ylim, tuple):
            ax.set_ylim(ylim)

    # Legend
    if show_legend and legend_handles:
        legend_kwargs = dict(
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_title,
            title_fontproperties={
                'weight': 'bold',
                'size': config.legend_title_fsize
            }
        )
        if legend_bbox:
            ax.legend(
                legend_handles,
                legend_labels,
                loc=legend_loc,
                bbox_to_anchor=legend_bbox,
                **legend_kwargs
            )
        else:
            ax.legend(
                legend_handles,
                legend_labels,
                loc=legend_loc,
                **legend_kwargs
            )

    return legend_handles, legend_labels


def create_multi_subplot_lineplots(
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
    wspace: Optional[float] = None,      # <-- NEW
    hspace: Optional[float] = None,      # <-- NEW
) -> str:

    """
    Create multiple line plot subplots with flexible configuration, mirroring
    the structure of `create_multi_subplot_boxplots`.

    Each entry in `subplot_configs` can contain:
        - 'item_column': str
        - 'category_column': str
        - 'value_column': str
        - 'categories': list
        - 'category_labels': list
        - 'ylabel': str
        - 'xlabel': str
        - 'title': str (subplot title)
        - 'filter': callable(DataFrame) -> DataFrame
        - 'ylim': tuple or float
        - 'target_lines': float or dict
        - 'rotation': int
        - 'show_legend': bool
        - 'legend_config': dict
        - 'show_grid': bool
        - 'ylabel_pad': float
        - 'agg_func': callable (for aggregation in prepare_lineplot_data)
        - 'line_styles': list of str
        - 'marker_styles': list of str
        - 'line_width': float
        - 'marker_size': float
    """
    # Read data
    df = read_csv_with_encoding(data_path)

    # Global filter
    if additional_filter is not None:
        df = additional_filter(df)

    # Layout
    n_subplots = len(subplot_configs)
    if n_rows is None and n_cols is None:
        n_cols = min(3, n_subplots)
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_rows is None:
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_cols is None:
        n_cols = (n_subplots + n_rows - 1) // n_rows

    # Figure size
    if fig_size is None:
        fig_size = (6 * n_cols, 4.5 * n_rows)

    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size)

    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if n_rows > 1 or n_cols > 1 else [axes]

    # Color palette
    max_categories = max(len(cfg['categories']) for cfg in subplot_configs)
    palette = sns.color_palette(palette_name, n_colors=max_categories)
    if color_reverse:
        palette = palette[::-1]

    all_legend_handles = []
    all_legend_labels = []

    # Build each subplot
    for idx, subplot_config in enumerate(subplot_configs):
        ax = axes[idx]

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
        rotation = subplot_config.get('rotation', 15)
        show_legend_subplot = subplot_config.get('show_legend', not shared_legend)
        legend_config = subplot_config.get('legend_config', {})
        show_grid = subplot_config.get('show_grid', False)
        ylabel_pad = subplot_config.get('ylabel_pad', None)
        agg_func = subplot_config.get('agg_func', np.mean)
        line_styles = subplot_config.get('line_styles', None)
        marker_styles = subplot_config.get('marker_styles', None)
        line_width = subplot_config.get('line_width', 1.5)
        marker_size = subplot_config.get('marker_size', 4.0)
        show_ylabel_subplot = subplot_config.get('show_ylabel', True)
        show_y_ticks = subplot_config.get('show_y_ticks', True)


        # Subplot-specific filter
        df_subplot = df.copy()
        if subplot_filter is not None:
            df_subplot = subplot_filter(df_subplot)

        df_subplot[item_column] = df_subplot[item_column].astype(str)
        df_subplot[category_column] = df_subplot[category_column].astype(str)

        # Sorted x items
        x_items = np.sort(df_subplot[item_column].unique())

        # Prepare data
        line_data = prepare_lineplot_data(
            df_subplot,
            x_items,
            item_column,
            categories,
            category_column,
            value_column,
            agg_func=agg_func
        )

        subplot_palette = palette[:len(categories)]

        # Plot
        handles, labels = plot_line_group(
            ax,
            line_data,
            categories,
            subplot_palette,
            config,
            show_ylabel=show_ylabel_subplot,
            ylabel=ylabel,
            xlabel=xlabel,
            x_tick_labels=x_items,
            rotation=rotation,
            ylim=ylim,
            show_legend=show_legend_subplot,
            legend_loc=legend_config.get('loc', 'best'),
            legend_bbox=legend_config.get('bbox', None),
            legend_title=legend_config.get('title', None),
            category_labels=category_labels,
            show_grid=show_grid,
            ylabel_pad=ylabel_pad,
            line_styles=line_styles,
            marker_styles=marker_styles,
            line_width=line_width,
            marker_size=marker_size,
            target_lines=target_lines,
            show_y_ticks=show_y_ticks,  # <-- NEW
        )

        # Title
        if subplot_title:
            ax.set_title(
                subplot_title,
                fontsize=config.title_fsize,
                fontweight='bold'
            )

        # Shared legend: store from first subplot
        if shared_legend and idx == 0:
            all_legend_handles = handles
            all_legend_labels = labels

    # Hide unused axes
    for idx in range(n_subplots, len(axes)):
        axes[idx].set_visible(False)

    # Shared legend (categories only; skip target lines)
    if shared_legend and all_legend_handles:
        legend_config = shared_legend_config or {}
        # Assume first len(categories) handles are the category lines
        n_categories = len(subplot_configs[0]['categories'])
        category_handles = all_legend_handles[:n_categories]
        category_labels = all_legend_labels[:n_categories]

        fig.legend(
            handles=category_handles,
            labels=category_labels,
            loc=legend_config.get('loc', 'upper center'),
            bbox_to_anchor=legend_config.get('bbox', (0.5, 0.98)),
            ncol=legend_config.get('ncol', len(category_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_config.get('title', None),
            title_fontproperties={
                'weight': 'bold',
                'size': config.legend_title_fsize
            },
            framealpha=legend_config.get('framealpha', 1.0),
            facecolor=legend_config.get('facecolor', 'white')
        )

    # Separate shared target line legend
    if shared_target_line_legend:
        target_line_value = shared_target_line_legend.get('value', 30)
        target_line_label = shared_target_line_legend.get('label', f'Target ({target_line_value})')

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

    # Figure title
    if figure_title:
        fig.suptitle(
            figure_title,
            fontsize=config.title_fsize + 2,
            fontweight='bold',
            y=1.02
        )

    if tight_layout_rect:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()

    # Optional manual spacing
    if wspace is not None or hspace is not None:
        fig.subplots_adjust(
            wspace=wspace if wspace is not None else plt.rcParams['figure.subplot.wspace'],
            hspace=hspace if hspace is not None else plt.rcParams['figure.subplot.hspace'],
        )

    # Save
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path


# ============================================================
# Convenience functions (single + comparison line plots)
# ============================================================

def create_single_lineplot(
    data_path: str,
    config: PlotConfig,
    **kwargs
) -> str:
    """
    Convenience function for a single lineplot.

    Required kwargs:
        - category_column
        - value_column
        - categories
        - output_filename

    Optional kwargs largely mirror create_single_boxplot.
    """
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
        'rotation': kwargs.get('rotation', 15),
        'show_legend': True,
        'legend_config': {
            'loc': kwargs.get('legend_loc', 'best'),
            'bbox': kwargs.get('legend_bbox', None),
            'title': kwargs.get('legend_title', None),
            'ncol': kwargs.get('legend_ncol', 1),
            'framealpha': kwargs.get('legend_framealpha', 1.0),
            'facecolor': kwargs.get('legend_facecolor', 'white'),
        },
        'show_grid': kwargs.get('show_grid', False),
        'ylabel_pad': kwargs.get('ylabel_pad', None),
        'agg_func': kwargs.get('agg_func', np.mean),
        'line_styles': kwargs.get('line_styles', None),
        'marker_styles': kwargs.get('marker_styles', None),
        'line_width': kwargs.get('line_width', 1.5),
        'marker_size': kwargs.get('marker_size', 4.0),
    }

    return create_multi_subplot_lineplots(
        data_path=data_path,
        config=config,
        subplot_configs=[subplot_config],
        output_filename=kwargs['output_filename'],
        fig_size=kwargs.get('fig_size', config.fig_size),
        additional_filter=kwargs.get('additional_filter', None),
        color_reverse=kwargs.get('color_reverse', True),
        palette_name=kwargs.get('palette_name', 'gist_earth')
    )


def create_comparison_lineplots(
    data_path: str,
    config: PlotConfig,
    comparison_column: str,
    comparison_values: List[Any],
    **kwargs
) -> str:
    """
    Convenience function for side-by-side comparison lineplots,
    similar to `create_comparison_boxplots`.

    Required kwargs:
        - category_column
        - value_column
        - categories
        - output_filename
    """
    subplot_configs = []

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
            'rotation': kwargs.get('rotation', 15),
            'show_legend': False,  # shared legend
            'show_grid': kwargs.get('show_grid', False),
            'ylabel_pad': kwargs.get('ylabel_pad', None),
            'agg_func': kwargs.get('agg_func', np.mean),
            'line_styles': kwargs.get('line_styles', None),
            'marker_styles': kwargs.get('marker_styles', None),
            'line_width': kwargs.get('line_width', 1.5),
            'marker_size': kwargs.get('marker_size', 4.0),
        }
        subplot_configs.append(subplot_config)

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': kwargs.get('legend_bbox', (0.5, 0.98)),
        'ncol': kwargs.get('legend_ncol', 1),
        'title': kwargs.get('legend_title', None),
        'framealpha': kwargs.get('legend_framealpha', 1.0),
        'facecolor': kwargs.get('legend_facecolor', 'white'),
    }

    tight_layout_rect = kwargs.get('tight_layout_rect', None)

    # Separate legend for target lines, if any
    shared_target_line_legend = None
    if target_lines is not None:
        if isinstance(target_lines, (int, float)):
            shared_target_line_legend = {
                'value': target_lines,
                'label': f'Epoch size ({target_lines}s)',
                'bbox': kwargs.get('legend_bbox2', (0.5, 0.92))
            }
        elif isinstance(target_lines, dict):
            first_label = list(target_lines.keys())[0]
            first_value = target_lines[first_label]
            shared_target_line_legend = {
                'value': first_value,
                'label': first_label,
                'bbox': kwargs.get('legend_bbox2', (0.5, 0.92))
            }

    return create_multi_subplot_lineplots(
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
        tight_layout_rect=tight_layout_rect,
    )


def plot_pruning_scatter_double(
    data_path: str,
    config: PlotConfig, output_filename, category_labels,
    category_column: str = "isSuccessorsLimited",
    group_column: str = "Instance",
    metrics: tuple = ("#LGenerated", "#LDominated"),
    ride_w2_values: tuple = (0.0, 0.5),
    categories: tuple = ("noPruning", "pruneNodes", "pruneArcs", "discardSuboptimalPath"),
    palette=None,
    ylabel: str = '# Labels(Generated/LDominated)',
    color_reverse=True,
    log_scale=True,
    rotation=15,
    ylim: tuple = (2_000_000, 2_000_000_000),
    marker_generated: str = "o",
    marker_dominated: str = "s",
    alpha_fill: float = 0.20):

    df = read_csv_with_encoding(data_path)

    # Auto palette
    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=len(categories))

    if color_reverse:
        palette = palette[::-1]

    colors = {cat: palette[i] for i, cat in enumerate(categories)}
    labels = {cat: category_labels[i] for i, cat in enumerate(categories)}

    gen_metric, dom_metric = metrics

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=config.fig_size)

    # ---------------------------------------------------------
    # Internal helper for plotting each axis
    # ---------------------------------------------------------
    def plot_on_axis(ax, df_filtered, ride_w2_value):
        grouped = (
            df_filtered.groupby([group_column, category_column])
            .agg({gen_metric: "mean", dom_metric: "mean"})
            .reset_index()
        )

        # Pivot to MultiIndex columns
        pivot_df = grouped.pivot(index=group_column, columns=category_column)

        # Axis style
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        # Plot each category
        for cat in categories:
            if cat not in pivot_df.columns.levels[1]:
                continue

            y_gen = pivot_df[(gen_metric, cat)]
            y_dom = pivot_df[(dom_metric, cat)]

            ax.plot(
                pivot_df.index,
                y_gen,
                label=f"{cat} ({gen_metric})",
                color=colors[cat],
                linestyle="-",
                marker=marker_generated,
            )
            ax.plot(
                pivot_df.index,
                y_dom,
                label=f"{cat} ({dom_metric})",
                color=colors[cat],
                linestyle=":",
                marker=marker_dominated,
            )

            ax.fill_between(
                pivot_df.index,
                y_gen,
                y_dom,
                color=colors[cat],
                alpha=alpha_fill,
            )

        # Axis labels / scales
        if log_scale:
            ax.set_yscale('log')
        ax.set_ylim(ylim)
        ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)
        ax.set_xticklabels(pivot_df.index, rotation=rotation)
        ax.set_xlabel(
            f"$W_{{delay}}$ = {ride_w2_value}",
            fontweight="bold",
            fontsize=config.axis_label_fsize,
        )

    # ---------------------------------------------------------
    # LEFT subplot (W_delay = ride_w2_values[0])
    # ---------------------------------------------------------
    df_left = df[df['Ride_W2'] == ride_w2_values[0]]
    plot_on_axis(ax_left, df_left, ride_w2_values[0])
    ax_left.set_ylabel(ylabel, fontweight="bold", fontsize=config.axis_label_fsize,)

    # ---------------------------------------------------------
    # RIGHT subplot (W_delay = ride_w2_values[1])
    # ---------------------------------------------------------
    df_right = df[df['Ride_W2'] == ride_w2_values[1]]
    plot_on_axis(ax_right, df_right, ride_w2_values[1])

    # ---------------------------------------------------------
    # LEGENDS
    # ---------------------------------------------------------

    # Legend 1: pruning strategies
    legend_pruning = [
        mlines.Line2D([], [], color=colors[cat], linewidth=3, label=labels[cat])
        for cat in categories
    ]

    fig.legend(
        handles=legend_pruning,
        loc="upper center",
        bbox_to_anchor=(0.40, 0.99),
        ncol=2,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Pruning Strategies",
        title_fontproperties={"weight": "bold", "size": config.legend_title_fsize},
    )

    # Legend 2: meaning of markers
    legend_stats = [
        mlines.Line2D([], [], color="black", linestyle="-", marker=marker_generated, label=gen_metric),
        mlines.Line2D([], [], color="black", linestyle=":", marker=marker_dominated, label=dom_metric),
    ]

    fig.legend(
        handles=legend_stats,
        loc="upper left",
        bbox_to_anchor=(0.68, 0.95),
        fontsize=config.legend_fsize,
        edgecolor="none",
        labelspacing=0.45,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.86])
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path


def plot_pruning_scatter_single(
    data_path: str,
    config: PlotConfig,
    output_filename,
    category_labels,
    category_column: str = "isSuccessorsLimited",
    group_column: str = "Instance",
    metrics: tuple = ("#LGenerated", "#LDominated"),
    ride_w2_values: tuple = (0.0, 0.5),
    categories: tuple = ("noPruning", "pruneNodes", "pruneArcs", "discardSuboptimalPath"),
    palette=None,
    ylabel: str = '# Labels(Generated/LDominated)',
    color_reverse=True,
    log_scale=True,
    rotation=15,
    ylim: tuple = (2_000_000, 2_000_000_000),
    marker_generated: str = "o",
    marker_dominated: str = "s",
    alpha_fill: float = 0.20):

    df = read_csv_with_encoding(data_path)

    # Auto palette
    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=len(categories))

    if color_reverse:
        palette = palette[::-1]

    colors = {cat: palette[i] for i, cat in enumerate(categories)}
    labels = {cat: category_labels[i] for i, cat in enumerate(categories)}

    gen_metric, dom_metric = metrics

    # Single axis instead of (ax_left, ax_right)
    fig, ax = plt.subplots(1, 1, figsize=(3,4))

    # ---------------------------------------------------------
    # Internal helper for plotting each axis
    # ---------------------------------------------------------
    def plot_on_axis(ax, df_filtered, ride_w2_value):
        grouped = (
            df_filtered.groupby([group_column, category_column])
            .agg({gen_metric: "mean", dom_metric: "mean"})
            .reset_index()
        )

        # Pivot to MultiIndex columns
        pivot_df = grouped.pivot(index=group_column, columns=category_column)

        # Axis style
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        # Plot each category
        for cat in categories:
            if cat not in pivot_df.columns.levels[1]:
                continue

            y_gen = pivot_df[(gen_metric, cat)]
            y_dom = pivot_df[(dom_metric, cat)]

            ax.plot(
                pivot_df.index,
                y_gen,
                label=f"{cat} ({gen_metric})",
                color=colors[cat],
                linestyle="-",
                marker=marker_generated,
                markersize = 5,
            )
            ax.plot(
                pivot_df.index,
                y_dom,
                label=f"{cat} ({dom_metric})",
                color=colors[cat],
                linestyle=":",
                marker=marker_dominated,
                markersize=5,
            )

            ax.fill_between(
                pivot_df.index,
                y_gen,
                y_dom,
                color=colors[cat],
                alpha=alpha_fill,
            )

        # Axis labels / scales
        if log_scale:
            ax.set_yscale('log')
        ax.set_ylim(ylim)
        ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)
        ax.set_xticklabels(pivot_df.index, rotation=rotation)
        ax.set_xlabel(
            "Instances",
            fontweight="bold",
            fontsize=config.axis_label_fsize,
        )

    # ---------------------------------------------------------
    # SINGLE plot (W_delay = ride_w2_values[0])
    # ---------------------------------------------------------
    df_left = df[df['Ride_W2'] == ride_w2_values[0]]
    plot_on_axis(ax, df_left, ride_w2_values[0])
    ax.set_ylabel(ylabel, fontweight="bold", fontsize=config.axis_label_fsize)

    # ---------------------------------------------------------
    # LEGENDS
    # ---------------------------------------------------------

    # Legend 1: pruning strategies
    legend_pruning = [
        mlines.Line2D([], [], color=colors[cat], linewidth=3, label=labels[cat])
        for cat in categories
    ]

    fig.legend(
        handles=legend_pruning,
        loc="upper left",
        bbox_to_anchor=(0.08, 0.95),
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Pruning Strategies",
        title_fontproperties={"weight": "bold", "size": config.legend_title_fsize},
        handlelength=1.2,
    )

    # Legend 2: meaning of markers
    legend_stats = [
        mlines.Line2D([], [], color="black", linestyle="-", marker=marker_generated, markersize = 5, label='#Generated'),
        mlines.Line2D([], [], color="black", linestyle=":", marker=marker_dominated, markersize = 5, label='#Dominated'),
    ]

    fig.legend(
        handles=legend_stats,
        loc="upper left",
        bbox_to_anchor=(0.5, 0.91),
        fontsize=config.legend_fsize,
        edgecolor="none",
        labelspacing=0.45,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.76])
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path



def plot_grouped_lines(
    data_path: str,
    config: PlotConfig,
    output_filename: str,
    category_labels,
    category_column: str = "object_category",
    group_column: str = "Instance_category",
    instance_column: str = "Instance",
    value_column: str = "wait/req",
    categories: tuple = None,
    palette=None,
    ylabel: str = "Sum of wait / req",
    title: str | None = None,
    rotation: int = 45,
    log_scale: bool = False,
    ylim: tuple | None = None,
):
    """
    Excel-like grouped line chart with group labels once and vertical separators.
    """

    # ---------------------------------------------------------
    # Read & prepare data
    # ---------------------------------------------------------
    df = read_csv_with_encoding(data_path)

    # Determine category order
    if categories is None:
        categories = tuple(df[category_column].unique())
    if len(categories) != len(category_labels):
        raise ValueError(
            f"len(category_labels)={len(category_labels)} "
            f"!= len(categories)={len(categories)}"
        )

    # Color palette
    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=len(categories))
    colors = {cat: palette[i] for i, cat in enumerate(categories)}
    labels = {cat: category_labels[i] for i, cat in enumerate(categories)}

    # Sort data so groups and instances appear in order
    df_sorted = df.sort_values([group_column, instance_column])

    # Pivot into wide format: index = (group, instance), columns = categories
    pivot = df_sorted.pivot_table(
        index=[group_column, instance_column],
        columns=category_column,
        values=value_column,
    )

    # Flatten index to get ordered lists of instances and groups
    pairs = pivot.index.to_frame(index=False)
    x_labels = pairs[instance_column].tolist()
    groups = pairs[group_column].tolist()

    x = np.arange(len(x_labels))

    # ---------------------------------------------------------
    # Create figure / axis
    # ---------------------------------------------------------
    fig, ax = plt.subplots(1, 1, figsize=config.fig_size)

    # Axis style (match your template)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # ---------------------------------------------------------
    # Plot lines, one per category
    # ---------------------------------------------------------
    for cat in categories:
        if cat not in pivot.columns:
            continue
        y = pivot[cat].to_numpy()
        ax.plot(
            x,
            y,
            marker="o",
            linewidth=2,
            label=labels[cat],
            color=colors[cat],
        )

    # X axis: instance labels only
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels, rotation=rotation, ha="right")
    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)

    # Y axis
    if log_scale:
        ax.set_yscale("log")
    if ylim is not None:
        ax.set_ylim(ylim)
    ax.set_ylabel(ylabel, fontweight="bold", fontsize=config.axis_label_fsize)

    if title is not None:
        ax.set_title(title, fontsize=config.title_fsize, fontweight="bold")

    # ---------------------------------------------------------
    # Group labels once + vertical separators
    # ---------------------------------------------------------
    group_order = list(pairs[group_column].drop_duplicates())
    group_sizes = [(groups.count(g)) for g in group_order]

    centers = []
    boundaries = []
    pos = 0
    for size in group_sizes:
        centers.append(pos + size / 2 - 0.5)
        pos += size
        boundaries.append(pos - 0.5)
    boundaries = boundaries[:-1]  # separators *between* groups only

    # Group labels below x-axis
    for center, gname in zip(centers, group_order):
        ax.text(
            center,
            -0.20,
            gname,
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=config.tick_label_fsize,
        )

    # Vertical separator lines
    for b in boundaries:
        ax.axvline(b, color="0.5", linewidth=1)

    ax.legend(
        fontsize=config.legend_fsize,
        edgecolor=config.legend_edgecolor,
        frameon=True,
        title_fontsize=config.legend_title_fsize,
    )

    # ---------------------------------------------------------
    # Save & return path
    # ---------------------------------------------------------
    plt.subplots_adjust(bottom=0.25)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path


def create_grouped_lineplot(
    data_path: str,
    config: PlotConfig,
    *,
    # core data choices
    value_column: str,
    category_column: str,
    categories: List[Any],
    group_column: str,
    groups: List[Any],
    # labels
    output_filename: str,
    ylabel: str,
    category_labels: Optional[List[str]] = None,
    group_labels: Optional[List[str]] = None,

    # basic formatting
    item_column: str = "Instance",
    ylim: Optional[Tuple[float, float]] = None,
    agg_func: Callable = np.mean,
    marker_styles: Optional[List[str]] = None,
    line_width: float = 2.0,
    marker_size: float = 5.5,
    rotation: int = 20,

    # palette & style
    palette_name: str = "gist_earth",
    color_reverse: bool = False,
    gap_size: int = 1,
    show_grid: bool = False,

    # legend
    legend_title: Optional[str] = None,
    legend_loc: str = "upper left",

    # figure
    fig_size: Optional[Tuple[float, float]] = None,
    additional_filter: Optional[Callable] = None,
) -> str:
    """
    Generic template for grouped line plots on a single axis.

    - One line per `categories` value (e.g. objective settings)
    - X-axis is all items (e.g. Instances) grouped by `groups`
      (e.g. '1.Excess Supply', '2.Balanced Supply', ...).
    """

    # ---- read and optionally filter data ----
    df = read_csv_with_encoding(data_path)
    if additional_filter is not None:
        df = additional_filter(df)

    df[item_column] = df[item_column].astype(str)
    df[category_column] = df[category_column].astype(str)
    df[group_column] = df[group_column].astype(str)

    # ---- labels / markers defaults ----
    if category_labels is None:
        category_labels = [str(c) for c in categories]
    if group_labels is None:
        group_labels = [str(g) for g in groups]
    if marker_styles is None:
        marker_styles = ["o", "s", "D", "^", "v"]  # extend if needed

    # ---- colour palette ----
    palette = sns.color_palette(palette_name, n_colors=len(categories))
    if color_reverse:
        palette = palette[::-1]

    # ---- build ordered x-axis with groups and gaps ----
    x_items: List[str] = []
    group_ranges: List[Tuple[int, int]] = []   # (start_idx, end_idx) in x_positions
    x_positions: List[float] = []

    pointer = 0
    for i, g in enumerate(groups):
        df_g = df[df[group_column] == str(g)]
        inst_sorted = np.sort(df_g[item_column].unique()).tolist()
        if len(inst_sorted) == 0:
            continue  # skip empty groups

        if i > 0:
            pointer += gap_size  # gap before each group except first

        group_pos = np.arange(pointer, pointer + len(inst_sorted))
        x_positions.extend(group_pos)

        start = pointer
        end = pointer + len(inst_sorted) - 1
        group_ranges.append((start, end))

        x_items.extend(inst_sorted)
        pointer = end + 1

    x_positions = np.array(x_positions)

    # ---- aggregate line data using your helper ----
    line_data = prepare_lineplot_data(
        df=df,
        items=x_items,
        items_column=item_column,
        categories=categories,
        category_column=category_column,
        value_column=value_column,
        agg_func=agg_func,
    )

    # ---- figure / axis ----
    if fig_size is None:
        fig_size = config.fig_size if hasattr(config, "fig_size") else (6.5, 4)

    fig, ax = plt.subplots(figsize=fig_size)

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if show_grid:
        ax.grid(True, alpha=0.3, linestyle="--", axis="y")
        ax.set_axisbelow(True)

    # ---- plot each category ----
    legend_handles = []
    for i, cat in enumerate(categories):
        handle = ax.plot(
            x_positions,
            line_data[cat],
            label=category_labels[i],
            linestyle="-",
            marker=marker_styles[i % len(marker_styles)],
            markersize=marker_size,
            linewidth=line_width,
            color=palette[i % len(palette)],
            markerfacecolor=palette[i % len(palette)],
            markeredgecolor=palette[i % len(palette)],
            markeredgewidth=1.2,
        )[0]
        legend_handles.append(handle)

    # ---- axes labels & ticks ----
    ax.set_ylabel(
        ylabel,
        fontsize=config.axis_label_fsize,
        fontweight="bold",
    )

    ax.set_xticks(x_positions)
    ax.set_xticklabels(x_items, rotation=rotation, fontsize=config.tick_label_fsize_samll)

    if ylim is not None:
        ax.set_ylim(ylim)

    # ---- vertical separators between groups ----
    for i in range(len(group_ranges) - 1):
        # center of gap between end of group i and start of group i+1
        gap_center = (group_ranges[i][1] + group_ranges[i + 1][0]) / 2.0
        ax.axvline(
            x=gap_center,
            color="lightgray",
            linewidth=1.0,
            linestyle="--",
        )

    # ---- group labels under x-axis ----
    # leave space at bottom for labels
    fig.subplots_adjust(bottom=0.20, top=0.82)

    for (start, end), label in zip(group_ranges, group_labels):
        mid = (start + end) / 2.0

        ax.text(
            mid, -0.22, label,
            transform=ax.get_xaxis_transform(),
            ha='center',
            fontsize=config.axis_label_fsize_small,
            fontweight='bold'
        )

    # ---- legend ----
    ax.legend(
        handles=legend_handles,
        labels=category_labels,
        loc=legend_loc,
        edgecolor=config.legend_edgecolor,
        fontsize=config.legend_fsize,
        title=legend_title,
        title_fontproperties={
            "weight": "bold",
            "size": config.legend_title_fsize,
        },
        framealpha=1.0,
        labelspacing=0.8,
    )

    fig.tight_layout()

    save_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(save_path, bbox_inches="tight")
    plt.close(fig)

    return save_path
