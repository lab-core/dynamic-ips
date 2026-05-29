"""
Reusable histogram plotting functions, following the same template pattern
as visualize_box_plots.py. Use plot_histogram_group for low-level control,
create_single_histogram / create_multi_subplot_histograms / create_comparison_histograms
for common workflows.
"""
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from typing import Dict, List, Optional, Tuple, Any, Union

from Simulation.utilities import read_csv_with_encoding
from Visualization.plot_config import PlotConfig


def plot_histogram_group(
    ax: plt.Axes,
    data_df: pd.DataFrame,
    value_column: str,
    category_column: str,
    categories: List[str],
    palette: List[Any],
    config: PlotConfig,
    *,
    category_labels: Optional[List[str]] = None,
    highlight_categories: Optional[Union[set, list]] = None,
    highlight_alpha: float = 0.45,
    bin_width: float = 5,
    bins: Optional[np.ndarray] = None,
    xlim: Optional[Tuple[Optional[float], Optional[float]]] = None,
    log_scale: bool = True,
    xlabel: str = '',
    ylabel: str = 'Count',
    xtick_interval: Optional[float] = None,
    show_legend: bool = True,
    legend_loc: str = 'upper right',
    legend_title: Optional[str] = None,
    legend_ncol: int = 2,
    legend_order: Optional[List[str]] = None,
) -> Tuple[List, List]:
    """
    Draw step histograms on the given axis, with optional filled highlights
    for selected categories (matching the style in create_rebalance_histo_by_supply).

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Axis to plot on.
    data_df : DataFrame
        Long-format data with value_column and category_column.
    value_column : str
        Column name for the variable to bin (x-axis).
    category_column : str
        Column name for the series (e.g. policy / algorithm).
    categories : list
        Category values in draw order (for hue_order; last = on top).
    palette : list
        Colors for categories (same order as categories).
    config : PlotConfig
        Font sizes and styling.
    category_labels : list, optional
        Legend labels; if None, uses categories (same order as categories).
    highlight_categories : set or list, optional
        Categories to draw as filled + outline; others as step-only lines.
    highlight_alpha : float
        Alpha for filled highlight (default 0.45).
    bin_width : float
        Bin width if bins is not provided (default 5).
    bins : array-like, optional
        Bin edges; if None, computed from data and bin_width.
    xlim : tuple (x_min, x_max), optional
        (None, None) or (min, max); None means auto.
    log_scale : bool
        Use log scale for y-axis (default True).
    xlabel, ylabel : str
        Axis labels.
    xtick_interval : float, optional
        Interval for x-ticks; if None, auto from data.
    show_legend : bool
        Whether to show legend.
    legend_loc : str
        Legend location.
    legend_title : str, optional
        Legend title.
    legend_ncol : int
        Number of columns in legend (default 2).
    legend_order : list, optional
        Category values in the order to show in the legend. If None, uses categories order.

    Returns
    -------
    legend_handles : list
    legend_labels : list
    """
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    highlight_set = set(highlight_categories) if highlight_categories else set()
    policy_colors = dict(zip(categories, palette))
    labels = category_labels if category_labels is not None else list(categories)

    # Bins
    if bins is None:
        max_val = data_df[value_column].max()
        bins = np.arange(0, max_val + bin_width, bin_width)

    other_df = data_df[~data_df[category_column].isin(highlight_set)]
    highlight_df = data_df[data_df[category_column].isin(highlight_set)]

    # Non-highlighted: step, no fill
    if not other_df.empty:
        sns.histplot(
            data=other_df,
            x=value_column,
            hue=category_column,
            hue_order=[p for p in categories if p not in highlight_set],
            bins=bins,
            element="step",
            fill=False,
 #           linewidth=0.8,
            palette={p: policy_colors[p] for p in categories if p not in highlight_set},
            common_norm=False,
            ax=ax,
            legend=False,
        )

    # Highlighted: fill then outline
    for hp in [p for p in categories if p in highlight_set]:
        hp_df = highlight_df[highlight_df[category_column] == hp]
        if hp_df.empty:
            continue
        sns.histplot(
            data=hp_df,
            x=value_column,
            bins=bins,
            element="step",
            fill=True,
   #         linewidth=0.0,
            alpha=highlight_alpha,
            color=policy_colors[hp],
            common_norm=False,
            ax=ax,
        )
  #      sns.histplot(
  #          data=hp_df,
  #          x=value_column,
  #          bins=bins,
  #          element="step",
  #          fill=False,
  #          linewidth=1.5,
  #          color=policy_colors[hp],
  #          common_norm=False,
  #          ax=ax,
  #          legend=False,
  #      )

    if xlim:
        x_min, x_max = xlim
        if x_min is not None:
            ax.set_xlim(left=x_min)
        if x_max is not None:
            ax.set_xlim(right=x_max)
    else:
        ax.set_xlim(left=0)

    if xtick_interval is not None:
        _, x_max = ax.get_xlim()
        ax.set_xticks(np.arange(0, x_max + 1, xtick_interval))

    ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.set_ylabel(ylabel, fontweight='bold', fontsize=config.axis_label_fsize)
    ax.tick_params(axis='both', which='major', labelsize=config.tick_label_fsize, length=3.8)

    if log_scale:
        ax.set_yscale('log')

    legend_handles = []
    legend_labels_out = []
    for pf, lab in zip(categories, labels):
        if pf in highlight_set:
            face_rgba = (*policy_colors[pf], highlight_alpha)
            legend_handles.append(
                Patch(
                    facecolor=face_rgba,
                    edgecolor=policy_colors[pf],
                    label=lab,
                )
            )
        else:
            legend_handles.append(
                Line2D([0], [0], color=policy_colors[pf], lw=1.8, label=lab)
            )
        legend_labels_out.append(lab)

    # Reorder legend by legend_order if provided
    if legend_order is not None:
        cat_to_handle_label = dict(zip(categories, zip(legend_handles, legend_labels_out)))
        legend_handles = [cat_to_handle_label[cat][0] for cat in legend_order if cat in cat_to_handle_label]
        legend_labels_out = [cat_to_handle_label[cat][1] for cat in legend_order if cat in cat_to_handle_label]

    if show_legend and legend_handles:
        ax.legend(
            handles=legend_handles,
            labels=legend_labels_out,
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            loc=legend_loc,
            ncol=legend_ncol,
            title=legend_title,
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
            framealpha=1.0,
            facecolor='white',
        )

    return legend_handles, legend_labels_out


def create_multi_subplot_histograms(
    config: PlotConfig,
    subplot_configs: List[Dict[str, Any]],
    output_filename: str,
    data_path: Optional[str] = None,
    output_dir: Optional[str] = None,
    fig_size: Optional[Tuple[float, float]] = None,
    n_rows: Optional[int] = None,
    n_cols: Optional[int] = None,
    shared_legend: bool = False,
    shared_legend_config: Optional[Dict[str, Any]] = None,
    additional_filter: Optional[Any] = None,
    color_reverse: bool = True,
    palette_name: str = "gist_earth",
    palette: Optional[List[Any]] = None,
    tight_layout_rect: Optional[List[float]] = None,
) -> str:
    """
    Create one figure with multiple histogram subplots (same pattern as
    create_multi_subplot_boxplots).

    Parameters
    ----------
    data_path : str
        Default path to CSV (used when a subplot config has no data_path/data_df).
    config : PlotConfig
        Plot configuration.
    subplot_configs : list of dict
        Each dict can contain:
        - data_path : optional path for this subplot only.
        - data_df : optional DataFrame for this subplot (overrides data_path).
        - value_column : column to bin (x-axis).
        - category_column : column for series (e.g. policy).
        - categories : list of category values.
        - category_labels : optional legend labels.
        - highlight_categories : optional set/list of categories to draw filled.
        - highlight_alpha : float (default 0.45).
        - bin_width : float (default 5).
        - bins : optional array of bin edges.
        - xlim : optional (x_min, x_max).
        - log_scale : bool (default True).
        - xlabel, ylabel : axis labels.
        - xtick_interval : optional float.
        - legend_loc, legend_title, legend_ncol.
        - filter : optional callable(df) -> filtered df for this subplot.
    output_filename : str
        Filename for the saved figure (e.g. 'idleTime_histo_ALL.pdf').
    output_dir : str, optional
        Directory to save; if None, uses dirname(data_path).
    fig_size, n_rows, n_cols : optional
        Figure layout; n_rows/n_cols inferred from len(subplot_configs) if omitted.
    shared_legend : bool
        If True, one legend for all subplots (from first subplot).
    shared_legend_config : dict, optional
        loc, bbox, ncol, title for shared legend.
    additional_filter : callable, optional
        Global filter applied to any DataFrame loaded from data_path.
    color_reverse : bool
        Reverse palette (default True).
    palette_name : str
        Seaborn palette name (default 'gist_earth').
    palette : list, optional
        Override palette (same order as categories).
    tight_layout_rect : list, optional
        [left, bottom, right, top] for tight_layout.

    Returns
    -------
    str
        Path to the saved figure.
    """
    if output_dir is None:
        output_dir = os.path.dirname(data_path) if data_path else os.getcwd()
    figure_path = os.path.join(output_dir, output_filename)

    # Load global data (optional when all subplots provide data_df)
    df_global = None
    df_cache: Dict[str, pd.DataFrame] = {}
    if data_path and os.path.isfile(data_path):
        df_global = read_csv_with_encoding(data_path)
        if additional_filter is not None:
            df_global = additional_filter(df_global)
        df_cache[data_path] = df_global

    n_subplots = len(subplot_configs)
    if n_rows is None and n_cols is None:
        n_cols = min(3, n_subplots)
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_rows is None:
        n_rows = (n_subplots + n_cols - 1) // n_cols
    elif n_cols is None:
        n_cols = (n_subplots + n_rows - 1) // n_rows

    if fig_size is None:
        fig_size = (3 * n_cols, 4 * n_rows)

    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size)
    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if n_rows > 1 or n_cols > 1 else [axes]

    max_categories = max(len(cfg['categories']) for cfg in subplot_configs)
    if palette is None:
        palette = sns.color_palette(palette_name, n_colors=max_categories)
    if color_reverse:
        palette = list(palette)[::-1]

    all_legend_handles = []
    all_legend_labels = []

    for idx, subplot_config in enumerate(subplot_configs):
        ax = axes[idx]
        value_column = subplot_config['value_column']
        category_column = subplot_config['category_column']
        categories = subplot_config['categories']
        category_labels = subplot_config.get('category_labels', categories)
        subplot_filter = subplot_config.get('filter', None)
        subplot_data_path = subplot_config.get('data_path', None)
        subplot_df = subplot_config.get('data_df', None)

        if subplot_df is not None:
            df_subplot = subplot_df.copy()
        else:
            path_to_use = subplot_data_path if subplot_data_path is not None else data_path
            if path_to_use and path_to_use in df_cache:
                df_subplot = df_cache[path_to_use].copy()
            elif path_to_use and os.path.isfile(path_to_use):
                df_tmp = read_csv_with_encoding(path_to_use)
                if additional_filter is not None:
                    df_tmp = additional_filter(df_tmp)
                df_cache[path_to_use] = df_tmp
                df_subplot = df_tmp.copy()
            elif df_global is not None:
                df_subplot = df_global.copy()
            else:
                raise ValueError(
                    "Subplot has no data_df and no valid data_path; "
                    "provide data_df in subplot_config or a valid data_path."
                )
        if subplot_filter is not None:
            df_subplot = subplot_filter(df_subplot)

        df_subplot[category_column] = df_subplot[category_column].astype(str)
        subplot_palette = palette[: len(categories)]

        handles, labels = plot_histogram_group(
            ax,
            df_subplot,
            value_column,
            category_column,
            categories,
            subplot_palette,
            config,
            category_labels=category_labels,
            highlight_categories=subplot_config.get('highlight_categories'),
            highlight_alpha=subplot_config.get('highlight_alpha', 0.45),
            bin_width=subplot_config.get('bin_width', 5),
            bins=subplot_config.get('bins'),
            xlim=subplot_config.get('xlim'),
            log_scale=subplot_config.get('log_scale', True),
            xlabel=subplot_config.get('xlabel', value_column),
            ylabel=subplot_config.get('ylabel', 'Count'),
            xtick_interval=subplot_config.get('xtick_interval'),
            show_legend=not shared_legend,
            legend_loc=subplot_config.get('legend_loc', 'upper right'),
            legend_title=subplot_config.get('legend_title'),
            legend_ncol=subplot_config.get('legend_ncol', 2),
            legend_order=subplot_config.get('legend_order'),
        )
        if shared_legend and idx == 0:
            all_legend_handles = handles
            all_legend_labels = labels

    for idx in range(n_subplots, len(axes)):
        axes[idx].set_visible(False)

    if shared_legend and all_legend_handles:
        leg_cfg = shared_legend_config or {}
        fig.legend(
            handles=all_legend_handles,
            labels=all_legend_labels,
            loc=leg_cfg.get('loc', 'upper center'),
            bbox_to_anchor=leg_cfg.get('bbox', (0.5, 0.98)),
            ncol=leg_cfg.get('ncol', len(all_legend_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=leg_cfg.get('title'),
            title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
            framealpha=leg_cfg.get('framealpha', 1.0),
            facecolor=leg_cfg.get('facecolor', 'white'),
        )

    if tight_layout_rect is not None:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()
    fig.savefig(figure_path, bbox_inches='tight')
    plt.close(fig)
    return figure_path


def create_histograms_for_metrics(
    data_df: pd.DataFrame,
    config: PlotConfig,
    output_dir: str,
    suffix: str,
    metric_configs: List[Tuple[str, str, str, str]],
    categories: List[str],
    category_labels: List[str],
    palette: List[Any],
    *,
    category_column: str = 'ReturnPolicy',
    highlight_categories: Optional[Union[set, list]] = None,
    legend_order: Optional[List[str]] = None,
    highlight_alpha: float = 0.45,
    bin_width: float = 5,
    log_scale: bool = True,
    xtick_interval: Optional[float] = 30,
    legend_title: Optional[str] = None,
    legend_ncol: int = 2,
    ylabel: str = 'Number of Vehicles',
    xlim_by_metric: Optional[Dict[str, Tuple[Optional[float], Optional[float]]]] = None,
) -> List[str]:
    """
    Create and save one histogram per metric (e.g. IdleTime, WorkingTime, ActiveTime).
    Each metric config is (value_column, xlabel, fname_base, legend_loc).

    Returns
    -------
    List of paths to saved figures.
    """
    paths = []
    for value_column, xlabel, fname_base, legend_loc in metric_configs:
        xlim = None
        if xlim_by_metric is not None and value_column in xlim_by_metric:
            xlim = xlim_by_metric[value_column]
        path = create_single_histogram(
            data_df=data_df,
            config=config,
            output_filename=f'{fname_base}_{suffix}.pdf',
            output_dir=output_dir,
            value_column=value_column,
            category_column=category_column,
            categories=categories,
            category_labels=category_labels,
            highlight_categories=highlight_categories,
            highlight_alpha=highlight_alpha,
            xlabel=xlabel,
            ylabel=ylabel,
            legend_title=legend_title,
            legend_loc=legend_loc,
            legend_ncol=legend_ncol,
            bin_width=bin_width,
            xlim=xlim,
            log_scale=log_scale,
            xtick_interval=xtick_interval,
            palette=palette,
            legend_order=legend_order,
        )
        paths.append(path)
    return paths


def create_single_histogram(
    data_path: Optional[str] = None,
    data_df: Optional[pd.DataFrame] = None,
    config: Optional[PlotConfig] = None,
    output_filename: str = 'histogram.pdf',
    output_dir: Optional[str] = None,
    **kwargs: Any,
) -> str:
    """
    Convenience function for a single histogram figure (same pattern as
    create_single_boxplot). Either data_path or data_df must be provided.

    kwargs are passed as a single subplot config to create_multi_subplot_histograms:
    value_column, category_column, categories, category_labels, xlabel, ylabel,
    highlight_categories, highlight_alpha, bin_width, xlim, log_scale, etc.
    """
    if config is None:
        config = PlotConfig()
    if data_df is not None:
        if output_dir is None and data_path:
            output_dir = os.path.dirname(data_path)
        elif output_dir is None:
            output_dir = os.getcwd()
        subplot_config = {
            'data_df': data_df,
            'value_column': kwargs['value_column'],
            'category_column': kwargs['category_column'],
            'categories': kwargs['categories'],
            'category_labels': kwargs.get('category_labels', kwargs['categories']),
            'xlabel': kwargs.get('xlabel', kwargs['value_column']),
            'ylabel': kwargs.get('ylabel', 'Count'),
            'highlight_categories': kwargs.get('highlight_categories'),
            'highlight_alpha': kwargs.get('highlight_alpha', 0.45),
            'bin_width': kwargs.get('bin_width', 5),
            'xlim': kwargs.get('xlim'),
            'log_scale': kwargs.get('log_scale', True),
            'legend_loc': kwargs.get('legend_loc', 'upper right'),
            'legend_title': kwargs.get('legend_title'),
            'legend_ncol': kwargs.get('legend_ncol', 2),
            'xtick_interval': kwargs.get('xtick_interval'),
            'legend_order': kwargs.get('legend_order'),
        }
        return create_multi_subplot_histograms(
            config=config,
            subplot_configs=[subplot_config],
            output_filename=output_filename,
            data_path=data_path,
            output_dir=output_dir,
            fig_size=kwargs.get('fig_size', config.fig_size),
            palette=kwargs.get('palette'),
            palette_name=kwargs.get('palette_name', 'gist_earth'),
            color_reverse=kwargs.get('color_reverse', True),
        )
    if data_path is None:
        raise ValueError("Either data_path or data_df must be provided.")
    if output_dir is None:
        output_dir = os.path.dirname(data_path)
    subplot_config = {
        'value_column': kwargs['value_column'],
        'category_column': kwargs['category_column'],
        'categories': kwargs['categories'],
        'category_labels': kwargs.get('category_labels', kwargs['categories']),
        'xlabel': kwargs.get('xlabel', kwargs['value_column']),
        'ylabel': kwargs.get('ylabel', 'Count'),
        'highlight_categories': kwargs.get('highlight_categories'),
        'highlight_alpha': kwargs.get('highlight_alpha', 0.45),
        'bin_width': kwargs.get('bin_width', 5),
        'xlim': kwargs.get('xlim'),
        'log_scale': kwargs.get('log_scale', True),
        'legend_loc': kwargs.get('legend_loc', 'upper right'),
        'legend_title': kwargs.get('legend_title'),
        'legend_ncol': kwargs.get('legend_ncol', 2),
        'xtick_interval': kwargs.get('xtick_interval'),
        'legend_order': kwargs.get('legend_order'),
    }
    return create_multi_subplot_histograms(
        config=config,
        subplot_configs=[subplot_config],
        output_filename=output_filename,
        data_path=data_path,
        output_dir=output_dir,
        fig_size=kwargs.get('fig_size', config.fig_size),
        palette=kwargs.get('palette'),
        palette_name=kwargs.get('palette_name', 'gist_earth'),
        color_reverse=kwargs.get('color_reverse', True),
    )


def create_comparison_histograms(
    data_path: str,
    config: PlotConfig,
    comparison_column: str,
    comparison_values: List[Any],
    output_filename: str,
    **kwargs: Any,
) -> str:
    """
    Convenience function for side-by-side histograms filtered by
    comparison_column (e.g. one subplot per vehicle category), with
    shared legend (same pattern as create_comparison_boxplots).
    """
    subplot_configs = []
    for value in comparison_values:
        subplot_config = {
            'value_column': kwargs['value_column'],
            'category_column': kwargs['category_column'],
            'categories': kwargs['categories'],
            'category_labels': kwargs.get('category_labels', kwargs['categories']),
            'xlabel': kwargs.get('xlabel', str(value)),
            'ylabel': kwargs.get('ylabel', 'Count') if len(subplot_configs) == 0 else '',
            'filter': lambda df, val=value: df[df[comparison_column] == val],
            'highlight_categories': kwargs.get('highlight_categories'),
            'highlight_alpha': kwargs.get('highlight_alpha', 0.45),
            'bin_width': kwargs.get('bin_width', 5),
            'xlim': kwargs.get('xlim'),
            'log_scale': kwargs.get('log_scale', True),
        }
        subplot_configs.append(subplot_config)

    shared_legend_config = {
        'loc': 'upper center',
        'bbox': kwargs.get('legend_bbox', (0.5, 0.98)),
        'ncol': kwargs.get('legend_ncol', 2),
        'title': kwargs.get('legend_title'),
    }
    return create_multi_subplot_histograms(
        config=config,
        subplot_configs=subplot_configs,
        output_filename=output_filename,
        data_path=data_path,
        fig_size=kwargs.get('fig_size'),
        n_rows=1,
        n_cols=len(comparison_values),
        shared_legend=True,
        shared_legend_config=shared_legend_config,
        additional_filter=kwargs.get('additional_filter'),
        color_reverse=kwargs.get('color_reverse', True),
        palette=kwargs.get('palette'),
        palette_name=kwargs.get('palette_name', 'gist_earth'),
        tight_layout_rect=kwargs.get('tight_layout_rect'),
    )
