import constants as c
from typing import Union, Sequence, Mapping

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import MultipleLocator
import seaborn as sns
from matplotlib.axes import Axes

import os

from Simulation.utilities import darken_color, read_csv_with_encoding
from Visualization.plot_config import PlotConfig

Number = Union[int, float]

def prepare_grouped_data(df, group_columns, agg_column, agg_func='mean'):
    """
    Group and aggregate data for bar plots.

    Parameters:
    -----------
    df : DataFrame
        The filtered dataframe
    group_columns : list
        Columns to group by (e.g., ['Instance', 'nbPick'])
    agg_column : str
        Column to aggregate
    agg_func : str or callable
        Aggregation function (default: 'mean')

    Returns:
    --------
    DataFrame : Pivoted dataframe ready for plotting
    """
    grouped = df.groupby(list(group_columns)).agg({
        agg_column: agg_func
    }).reset_index()

    # Pivot to get proper structure
    if len(group_columns) == 2:
        return grouped.pivot_table(
            index=group_columns[0],
            columns=group_columns[1],
            values=agg_column
        )
    elif len(group_columns) == 1:
        return grouped.set_index(group_columns[0])[agg_column].to_frame()
    else:
        return grouped.set_index(group_columns[0])

def _style_axis(ax: Axes, config: PlotConfig, show_grid: bool = False, grid_axis: str = "y",) -> None:
    """Apply common axis styling."""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # Configure ticks
    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)

    # Add grid if requested
    if show_grid:
        ax.grid(color="lightgray", linestyle="--", axis=grid_axis, alpha=0.5)

def _calculate_bar_positions(n_instances: int, n_bars: int, width: float = 0.1, cluster_spacing: float = 2.5,)-> np.ndarray:
    """
    Calculate positions for grouped bar charts.

    Parameters:
    -----------
    n_instances : int
        Number of instance groups
    n_bars : int
        Number of bars per instance
    width : float
        Width of each bar
    cluster_spacing : float
        Spacing between instance clusters

    Returns:
    --------
    array : Base positions for bar clusters
    """
    if n_instances == 0:
        return np.array([])
    return np.arange(n_instances) * (width * (n_bars + cluster_spacing))

def _plot_grouped_bars(ax, pivot_df, categories, palette, config,
                      width=0.1, cluster_spacing=2.5,
                      show_ylabel=True, ylabel='Value',
                      xlabel='Instances', rotation=25,
                      log_scale=False, ylim=None, xlim_padding=None,
                      target_line=None, grid=False):

    _style_axis(ax, config, show_grid=grid)

    # Handle categories as dict or list
    if isinstance(categories, dict):
        category_keys = list(categories.keys())
        category_labels = list(categories.values())
    else:
        category_keys = categories
        category_labels = categories

    if pivot_df.empty:
        return np.array([])

    # Calculate positions
    n_instances = len(pivot_df.index)
    n_bars = len(category_keys)
    positions = _calculate_bar_positions(n_instances, n_bars, width, cluster_spacing)

    # Plot bars for each category
    for i, cat in enumerate(category_keys):
        ax.bar(
            positions + i * width + width / 2,
            pivot_df[cat],
            width=width,
            label=category_labels[i] if isinstance(categories, dict) else cat,
            color=palette[i],
            edgecolor=darken_color(palette[i], factor=0.5)
        )

    # Add target line if specified
    if target_line is not None:
        ax.axhline(y=target_line, color='red', linestyle='--',
                   linewidth=1, label=f'Epoch size ({target_line}s)')

    # Set labels and ticks
    ax.set_xticks(positions + width * n_bars / 2)
 #   ax.set_xticklabels(c.customer_groups_labels, rotation=rotation, fontsize=config.tick_label_fsize)
    ax.set_xticklabels(pivot_df.index, rotation=rotation, fontsize=config.tick_label_fsize)

    if show_ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight='bold')

    ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)

    # Set scale
    if log_scale:
        ax.set_yscale('log')

    # Set limits
    if ylim is not None:
        if isinstance(ylim, (int, float)):
            ax.set_ylim(top=ylim)
        else:
            ax.set_ylim(ylim)

    if xlim_padding is None:
        xlim_padding = width * 2

    if len(positions) > 0:
        ax.set_xlim(positions[0] - xlim_padding,
                    positions[-1] + n_bars * width * 1.5)

    return positions


def _plot_single_category_bars(ax, values: Union[pd.Series, pd.DataFrame], color, config,
                              show_ylabel=True, ylabel='Value',
                              xlabel='Instances', rotation=25,
                              width=0.5, ylim=None,
                              target_line=None, grid=False,
                              major_locator=None):

    _style_axis(ax, config, show_grid=grid)

    # Get data (handle both DataFrame and Series)
    if isinstance(values, pd.DataFrame):
        if values.shape[1] != 1:
            raise ValueError(
                "values DataFrame must have exactly one column "
                "for _plot_single_category_bars."
            )
        series = values.iloc[:, 0]
    else:
        series = values

    x = np.arange(len(series))

    # Plot bars
    ax.bar(
        x,
        series.values,
        color=color,
        alpha=0.6,
        width=width,
        edgecolor=darken_color(color, 0.2)
    )

    # Add target line if specified
    if target_line is not None:
        ax.axhline(y=target_line, color='darkslategray', linewidth=1, linestyle='--')

    # Set labels and ticks
    ax.set_xticks(x)
    ax.set_xticklabels(series.index.astype(str), rotation=rotation,fontsize=config.tick_label_fsize)

    if show_ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight='bold')

    ax.set_xlabel(xlabel, fontweight='bold', fontsize=config.axis_label_fsize)

    # Set limits
    if ylim is not None:
        if isinstance(ylim, (int, float)):
            ax.set_ylim(top=ylim)
        else:
            ax.set_ylim(ylim)

    # Set major locator if specified
    if major_locator is not None:
        ax.yaxis.set_major_locator(MultipleLocator(major_locator))


def create_double_grouped_barplot(data_path, config: PlotConfig, output_filename,
                                  group_columns, agg_column, categories,
                                  category_labels, legend_title,
                                  ylabel, ride_w2_values: Sequence[Number] = (0.0, 0.5),
                                  color_reverse=True,
                                  width=0.1, cluster_spacing=2.5,
                                  rotation=25, log_scale=False,
                                  ylim_left=None, ylim_right=None,
                                  target_line=None, target_line_label='Target',
                                  additional_filter=None,
                                  bbox_anchor_legend1=(0.55, 0.99),
                                  bbox_anchor_legend2=(0.495, 0.86),
                                  agg_func='mean', grid=False,
                                  alpha=0.7):

    # Read data
    df = read_csv_with_encoding(data_path)

    # Apply additional filtering if provided
    if additional_filter is not None:
        df = additional_filter(df)

    # Handle categories as dict or list
    if isinstance(categories, Mapping):
        category_keys = list(categories.keys())
        default_labels = list(categories.values())
    else:
        category_keys = categories
        default_labels = category_keys

    if category_labels is None:
        category_labels = default_labels

    # Get palette
    palette = sns.color_palette("gist_earth", n_colors=len(category_keys))

    if color_reverse:
        palette = palette[::-1]

    # Apply alpha
    palette_with_alpha = [
        (r, g, b, alpha) if len(c) == 3 else (*c[:3], alpha)
        for c, (r, g, b) in zip(palette, palette)
    ]

    # Initialize figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=config.fig_size)

    # LEFT subplot
    df_left = df[df['Ride_W2'] == ride_w2_values[0]]
    pivot_left = prepare_grouped_data(df_left, group_columns, agg_column, agg_func)
    _plot_grouped_bars(
        ax1, pivot_left, category_keys, palette_with_alpha, config,
        width=width, cluster_spacing=cluster_spacing,
        show_ylabel=True, ylabel=ylabel,
        xlabel=f"$W_{{delay}}$ = {ride_w2_values[0]}",
        rotation=rotation, log_scale=log_scale,
        ylim=ylim_left, target_line=target_line, grid=grid
    )

    # RIGHT subplot
    df_right = df[df['Ride_W2'] == ride_w2_values[1]]
    pivot_right = prepare_grouped_data(df_right, group_columns, agg_column, agg_func)
    _plot_grouped_bars(
        ax2, pivot_right, categories, palette_with_alpha, config,
        width=width, cluster_spacing=cluster_spacing,
        show_ylabel=False, ylabel=ylabel,
        xlabel=f"$W_{{delay}}$ = {ride_w2_values[1]}",
        rotation=rotation, log_scale=log_scale,
        ylim=ylim_right, target_line=target_line, grid=grid
    )

    # Category legend
    darkened_palette = [darken_color(color, factor=0.5) for color in palette]
    legend_handles = [
        plt.Rectangle((0, 0), 1, 1,
                      facecolor=palette_with_alpha[i],
                      edgecolor=darkened_palette[i])
        for i in range(len(category_keys))
    ]

    # First legend: categories
    fig.legend(
        handles=legend_handles,
        labels=category_labels,
        loc='upper center',
        bbox_to_anchor=bbox_anchor_legend1,
        ncol=2,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title=legend_title,
        title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
        framealpha=1.0,
        facecolor='white'
    )

    # Second legend: target line (if specified)
    if target_line is not None:
        fig.legend(
            handles=[mlines.Line2D([], [], color='red', linestyle='--', lw=1.2)],
            labels=[target_line_label],
            loc='upper center',
            bbox_to_anchor=bbox_anchor_legend2,
            ncol=1,
            fontsize=config.legend_fsize,
            labelspacing=0.45,
            edgecolor="none",
        )

    # Adjust layout and save
    fig.tight_layout(rect=[0, 0, 1, 0.85])
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path


def create_double_relative_barplot(data_path, config: PlotConfig,
                                   group_column, category_column,
                                   value_column, output_filename,
                                   baseline_value, comparison_value,
                                   ylabel: str = "Relative difference (%)",
                                   ride_w2_values: Sequence[Number] = (0.0, 0.5),
                                   color='lightsteelblue',
                                   width=0.5, rotation=15,
                                   ylim_left=None, ylim_right=None,
                                   legend_label='Relative Difference(%)',
                                   bbox_anchor=(0.5, 0.99),
                                   major_locator=1,
                                   grid=False):

    # Read data
    df = read_csv_with_encoding(data_path)

    # Initialize figure
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=config.fig_size)

    def _relative_diff_for_subset(df_subset: pd.DataFrame) -> pd.Series:

        grouped = df_subset.groupby([group_column, category_column]).agg({
            value_column: 'mean'
        }).reset_index()

        pivot = grouped.pivot(
            index=group_column,
            columns=category_column,
            values=value_column
        )
        missing = {baseline_value, comparison_value} - set(pivot.columns)
        if missing:
            raise KeyError(f"Missing required categories in pivot: {missing}")

        # Calculate relative improvement
        relative_diff = (pivot[baseline_value] - pivot[comparison_value]) / pivot[baseline_value] * 100.0
        return relative_diff


    # LEFT subplot
    df_left = df[df['Ride_W2'] == ride_w2_values[0]]
    rel_left = _relative_diff_for_subset(df_left)
    _plot_single_category_bars(
        ax=ax_left,
        values=rel_left,
        color=color,
        config=config,
        show_ylabel=True,
        ylabel=ylabel,
        xlabel=f"$W_{{delay}}$ = {ride_w2_values[0]}",
        rotation=rotation,
        width=width,
        ylim=ylim_left,
        grid=grid,
        major_locator=major_locator,
    )
    ylim = ax_left.get_ylim()
    if ylim[0] < 0:
        ax_left.axhline(y=0, color="darkslategray", linewidth=1, linestyle="--")

    # RIGHT subplot
    df_right = df[df['Ride_W2'] == ride_w2_values[1]]
    rel_right = _relative_diff_for_subset(df_right)

    _plot_single_category_bars(
        ax=ax_right,
        values=rel_right,
        color=color,
        config=config,
        show_ylabel=False,
        ylabel=ylabel,
        xlabel=f"$W_{{delay}}$ = {ride_w2_values[1]}",
        rotation=rotation,
        width=width,
        ylim=ylim_right,
        grid=grid,
        major_locator=major_locator,
    )

    ylim = ax_right.get_ylim()
    if ylim[0] < 0:
        ax_right.axhline(y=0, color="darkslategray", linewidth=1, linestyle="--")

    # Create legend
    fig.legend(
        handles=[plt.Rectangle((0, 0), 1, 1,
                               facecolor=color,
                               alpha=0.6,
                               edgecolor=darken_color(color, 0.2))],
        labels=[legend_label],
        loc='upper center',
        bbox_to_anchor=bbox_anchor,
        ncol=1,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor
    )

    # Adjust layout and save
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path
