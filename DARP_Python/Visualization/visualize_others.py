import os
import constants as c
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import List, Optional, Tuple, Any
import seaborn as sns
from matplotlib.ticker import MultipleLocator
import matplotlib.lines as mlines

from Simulation.utilities import read_csv_with_encoding
from Visualization.plot_config import PlotConfig
from Visualization.visualize_box_plots import plot_boxplot_group, prepare_boxplot_data


def plot_matrix_heatmap(
    ax: plt.Axes,
    df: pd.DataFrame,
    row_column: str,
    col_column: str,
    value_column: str,
    config: PlotConfig,
    max_rows: int = 100,
    cmap: str = "viridis",
    xlabel: str = "",
    ylabel: str = "",
    title: str = "",
    cbar_label: Optional[str] = None,
    x_tick_step: Optional[int] = None,
    y_tick_step: Optional[int] = None,
    vmin: Optional[float] = None,
    vmax: Optional[float] = None,
    interpolation: str = "nearest",
) -> None:
    """
    Plot a single heatmap on the given axis.

    Parameters
    ----------
    ax : plt.Axes
        Axis to draw on.
    df : pd.DataFrame
        Data containing row, column and value columns.
    row_column : str
        Column used as heatmap rows (e.g., 'VehicleID').
    col_column : str
        Column used as heatmap columns (e.g., 'Epoch').
    value_column : str
        Column used as heatmap cell values (e.g., 'length').
    max_rows : int, default 100
        Maximum number of distinct row values to display.
        If more rows exist, sample 1 out of every (n_rows / max_rows).
    """
    vehicle_sums = df.groupby(row_column)[value_column].sum()
    sorted_rows = vehicle_sums.sort_values().index.to_numpy()

    n_rows = len(sorted_rows)

    # sample if needed
    if n_rows > max_rows:
        step = max(1, n_rows // max_rows)
        sampled_rows = sorted_rows[::step][:max_rows]
    else:
        sampled_rows = sorted_rows

    df_sub = df[df[row_column].isin(sampled_rows)].copy()

    # ----- Pivot to matrix: rows × columns -----
    pivot_df = df_sub.pivot(
        index=row_column,
        columns=col_column,
        values=value_column
    )

    # preserve sorted vehicle order explicitly
    pivot_df = pivot_df.reindex(index=sampled_rows)

    # also sort columns
    pivot_df = pivot_df.sort_index(axis=1)

    heatmap_data = pivot_df.to_numpy(dtype=float)

    # ----- Plot styling (similar minimalist style) -----
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    im = ax.imshow(
        heatmap_data,
        aspect="auto",
        interpolation=interpolation,
        cmap=cmap,
        vmin=vmin,  # <-- use vmin/vmax
        vmax=vmax,
    )

    # Colorbar
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.tick_params(labelsize=config.tick_label_fsize_samll)
    if cbar_label is not None:
        cbar.set_label(cbar_label, fontsize=config.axis_label_fsize_small, fontweight="bold")

    # Axis labels
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=config.axis_label_fsize, fontweight="bold")
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")

    # ---- Tick labels (downsampled if needed) ----
    n_cols = pivot_df.shape[1]
    n_rows_mat = pivot_df.shape[0]

    # X axis (columns)
    if n_cols > 0:
        if x_tick_step is None:
            x_tick_step = max(1, n_cols // 6)  # ~10 labels
        x_positions = np.arange(0, n_cols+1, x_tick_step)
        ax.set_xticks(x_positions)
        ax.set_xticklabels(
            x_positions,
            fontsize=config.tick_label_fsize,
            ha="right",
        )

    ax.set_yticklabels([])
    ax.set_yticks([])

    # Title
    if title:
        ax.set_title(title, fontsize=config.title_fsize, fontweight="bold")



def create_single_heatmap_figure(
    df: pd.DataFrame,
    config: PlotConfig,
    row_column: str,
    col_column: str,
    value_column: str,
    output_path: str,
    fig_size: Optional[Tuple[float, float]] = None,
    xlabel: str = "",
    ylabel: str = "",
    title: str = "",
    cbar_label: Optional[str] = None,
    max_rows: int = 100,
    cmap: str = "viridis",
    x_tick_step: Optional[int] = None,
    y_tick_step: Optional[int] = None,
    vmin: Optional[float] = None,
    vmax: Optional[float] = None,
    interpolation: str = "nearest",
) -> str:
    """
    Create a single heatmap figure and save it to output_path.
    """
    if fig_size is None:
        fig_size = config.fig_size

    fig, ax = plt.subplots(1, 1, figsize=fig_size)

    plot_matrix_heatmap(
        ax=ax,
        df=df,
        row_column=row_column,
        col_column=col_column,
        value_column=value_column,
        config=config,
        max_rows=max_rows,
        cmap=cmap,
        xlabel=xlabel,
        ylabel=ylabel,
        title=title,
        cbar_label=cbar_label,
        x_tick_step=x_tick_step,
        y_tick_step=y_tick_step,
        vmin = vmin,
        vmax = vmax,
        interpolation=interpolation,
    )

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path


def create_single_epoch_std_figure(
    df: pd.DataFrame,
    config: PlotConfig,
    epoch_column: str,
    value_column: str,
    output_path: str,
    fig_size: Optional[Tuple[float, float]] = None,
    xlabel: str = "",
    ylabel: str = "",
    title: str = "",
) -> str:
    """
    Compute the standard deviation of `value_column` for each epoch and plot as a line.

    Parameters
    ----------
    df : pd.DataFrame
        Data containing epoch and value columns.
    epoch_column : str
        Column name for epochs (e.g., 'Epoch').
    value_column : str
        Column whose std is computed per epoch (e.g., 'nbRequests').
    output_path : str
        Where to save the plot.
    """
    if fig_size is None:
        fig_size = config.fig_size

    # ---- Compute std per epoch ----
    std_per_epoch = (
        df.groupby(epoch_column)[value_column]
          .std()                         # std across vehicles within each epoch
          .sort_index()
    )

    epochs = std_per_epoch.index.to_numpy()
    std_values = std_per_epoch.to_numpy()

    # ---- Create figure ----
    fig, ax = plt.subplots(1, 1, figsize=fig_size)

    # Minimal styling, similar to your heatmap style
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    ax.plot(epochs, std_values, marker="o", linewidth=1.5)

    # Labels
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=config.axis_label_fsize, fontweight="bold")
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")

    # Title
    if title:
        ax.set_title(title, fontsize=config.title_fsize, fontweight="bold")

    # Ticks
    ax.tick_params(axis="both", labelsize=config.tick_label_fsize)

    # Optional grid
    ax.grid(True, linestyle="--", alpha=0.4)

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path


def plot_profile_mean_std_shaded(
    ax: plt.Axes,
    df,
    x_column: str,
    mean_column: str,
    std_column: str,
    group_column: str,
    groups: List[Any],
    config: PlotConfig,
    palette: Optional[List[Any]] = None,
    group_labels: Optional[List[str]] = None,
    alpha_fill: float = 0.22,
    linewidth: float = 2.0,
    xlabel: str = "",
    ylabel: str = "",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    x_major_step: Optional[float] = None,
    y_major_step: Optional[float] = None,
    grid: bool = True,
) -> Tuple[List[Any], List[str]]:

    # Basic axis styling
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if grid:
        ax.grid(True, color="lightgray", linestyle="--", alpha=0.5)
        ax.set_axisbelow(True)

    # Determine colors
    n_groups = len(groups)
    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=n_groups)[::-1]

    if group_labels is None:
        group_labels = [str(g) for g in groups]

    legend_handles = []
    legend_labels = []

    # Plot mean + shaded std for each group
    for i, g in enumerate(groups):
        sub_df = df[df[group_column] == g].sort_values(by=x_column)

        if sub_df.empty:
            continue

        x = sub_df[x_column]
        y_mean = sub_df[mean_column]
        y_std = sub_df[std_column]

        sem = y_std / 5

        # Shaded band
        ax.fill_between(
            x,
            y_mean - sem,
            y_mean + sem,
            color=palette[i],
            alpha=alpha_fill,
        )

        # Mean line
        line, = ax.plot(
            x,
            y_mean,
            color=palette[i],
            linewidth=linewidth,
        )

        legend_handles.append(line)
        legend_labels.append(group_labels[i])

    # Labels
    if show_ylabel and ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")
    if show_xlabel and xlabel:
        ax.set_xlabel(xlabel, fontsize=config.axis_label_fsize, fontweight="bold")

    # Limits
    if xlim: ax.set_xlim(*xlim)
    if ylim: ax.set_ylim(*ylim)

    # Ticks
    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)
    if x_major_step is not None:
        ax.xaxis.set_major_locator(MultipleLocator(x_major_step))
    if y_major_step is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_major_step))

    return legend_handles, legend_labels


def create_shaded_mean_std_plot(
    df,
    metric: str,
    label: str,
    test: str,
    config: PlotConfig,
    palette,
    output_folder: str,
):
    output_path = os.path.join(output_folder, f"{test}_{label.replace(' ','_')}_shaded.pdf")

    fig, ax = plt.subplots(figsize=(12, 5))

    plot_profile_mean_std_shaded(
        ax=ax,
        df=df,
        x_column='ElapsedTime',
        mean_column=f'mean_{metric}',
        std_column=f'std_{metric}',
        group_column='object_category',
        groups=c.OBJECTIVES,
        group_labels=c.OBJECTIVES_labels,
        config=config,
        palette=palette,
        ylabel=f"{label} (mean ± std)",
        xlabel="Elapsed Time (min)",
        xlim=(0, 120),
        x_major_step=15,
    )

    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path

def plot_profile_mean_quantile_shaded(
    ax: plt.Axes,
    df,
    x_column: str,
    center_column: str,          # e.g. "mean_nbCommitted"
    lower_q_column: str,         # e.g. "q25_nbCommitted"
    upper_q_column: str,         # e.g. "q75_nbCommitted"
    group_column: str,
    groups: List[Any],
    config: PlotConfig,
    palette: Optional[List[Any]] = None,
    group_labels: Optional[List[str]] = None,
    alpha_fill: float = 0.22,
    linewidth: float = 2.0,
    xlabel: str = "",
    ylabel: str = "",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    x_major_step: Optional[float] = None,
    y_major_step: Optional[float] = None,
    grid: bool = True,
):
    """
    Plot mean as a line and an inter-quantile band (e.g. 25–75%).
    """
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if grid:
        ax.grid(True, color="lightgray", linestyle="--", alpha=0.5)
        ax.set_axisbelow(True)

    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=len(groups))[::-1]

    if group_labels is None:
        group_labels = [str(g) for g in groups]

    legend_handles, legend_labels = [], []

    for i, g in enumerate(groups):
        sub_df = df[df[group_column] == g].sort_values(by=x_column)
        if sub_df.empty:
            continue

        x      = sub_df[x_column].values
        y_mean = sub_df[center_column].values
        y_low  = sub_df[lower_q_column].values
        y_high = sub_df[upper_q_column].values

        # shade between quantiles
        ax.fill_between(
            x,
            y_low,
            y_high,
            color=palette[i],
            alpha=alpha_fill,
        )

        # center line (mean)
        line, = ax.plot(
            x,
            y_mean,
            color=palette[i],
            linewidth=linewidth,
        )

        legend_handles.append(line)
        legend_labels.append(group_labels[i])

    # Labels & formatting
    if show_ylabel and ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")
    if show_xlabel and xlabel:
        ax.set_xlabel(xlabel, fontsize=config.axis_label_fsize, fontweight="bold")

    if xlim: ax.set_xlim(*xlim)
    if ylim: ax.set_ylim(*ylim)

    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)
    if x_major_step: ax.xaxis.set_major_locator(MultipleLocator(x_major_step))
    if y_major_step: ax.yaxis.set_major_locator(MultipleLocator(y_major_step))

    ax.legend(
        legend_handles,
        legend_labels,
        fontsize=config.legend_fsize,
        edgecolor=config.legend_edgecolor,
        loc="best",
    )

    return legend_handles, legend_labels



def create_pruning_labels_and_time_figure(
    data_path: str,
    config: PlotConfig,
    ride_w2_value: float = 0.0,
    output_filename: str = "pruning_labels_and_time.pdf",
    categories=("noPruning", "pruneNodes", "pruneArcs", "discardSuboptimalPath"),
    category_labels=('No pruning', 'Prune nodes', 'Prune arcs', 'Prune suboptimal path'),
    ylabel_scatter='Number of Labels',
    ylabel_boxplot='Epoch Runtime (s)',
    ylim_scatter=(2e6, 2e9),
    ylim_boxplot=(1150, 750),       # same as your existing call
    target_lines=30,
    rotation=15,
    color_reverse=True,
    palette_name="gist_earth",
    marker_generated="o",
    marker_dominated="s",
    alpha_fill=0.20,
    show_outliers=True,
):
    """
    Single figure with two subplots:
        left  -> labels (Generated / LDominated) for Ride_W2 == ride_w2_value
        right -> runtime boxplots for Ride_W2 == ride_w2_value
    """

    # ------------------------------------------------------------------
    # Load + filter data
    # ------------------------------------------------------------------
    df = read_csv_with_encoding(data_path)
    df = df[df["Ride_W2"] == ride_w2_value].copy()

    # ------------------------------------------------------------------
    # Shared palette / colors (same for both subplots)
    # ------------------------------------------------------------------
    palette = sns.color_palette(palette_name, n_colors=len(categories))
    if color_reverse:
        palette = palette[::-1]
    colors = {cat: palette[i] for i, cat in enumerate(categories)}
    labels = {cat: category_labels[i] for i, cat in enumerate(categories)}

    # ------------------------------------------------------------------
    # Create figure + axes
    # ------------------------------------------------------------------
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=config.fig_size)

    # ==============================================================
    # LEFT AXIS: SCATTER / LINE PLOT (#LGenerated / #LDominated)
    # ==============================================================

    gen_metric = "#LGenerated"
    dom_metric = "#LDominated"

    grouped = (
        df.groupby(["Instance", "isSuccessorsLimited"])
          .agg({gen_metric: "mean", dom_metric: "mean"})
          .reset_index()
    )
    pivot_df = grouped.pivot(index="Instance", columns="isSuccessorsLimited")

    # axis styling
    ax = ax_left
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    for cat in categories:
        # skip missing categories
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

    ax.set_ylabel(ylabel_scatter, fontweight="bold", fontsize=config.axis_label_fsize)
    ax.set_xlabel( 'Instances', fontweight="bold", fontsize=config.axis_label_fsize)
    ax.set_yscale("log")
    ax.set_ylim(ylim_scatter)
    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)
    ax.set_xticklabels(pivot_df.index, rotation=rotation)

    # ==============================================================
    # RIGHT AXIS: BOX PLOTS (Total time)
    # ==============================================================

    item_column = "Instance"
    category_column = "isSuccessorsLimited"
    value_column = "Total time"

    df_box = df.copy()
    df_box[item_column] = df_box[item_column].astype(str)
    df_box[category_column] = df_box[category_column].astype(str)

    x_items = np.sort(df_box[item_column].unique())

    # Prepare data for your existing helper
    plot_data = prepare_boxplot_data(
        df_box,
        x_items,
        item_column,
        categories,
        category_column,
        value_column,
    )


    handles_box, labels_box = plot_boxplot_group(
        ax_right,
        plot_data,
        categories,
        palette,
        config,
        show_outliers=show_outliers,
        show_ylabel=True,
        ylabel=ylabel_boxplot,
        width=0.35,
        gap_factors=[0, 0, 0, 0],
        target_lines=target_lines,
        xlabel='Instances',
        x_tick_labels=x_items,
        rotation=rotation,
    #    ylim=ylim_boxplot,
        show_legend=False,              # we’ll make a shared legend below
        legend_loc="best",
        legend_bbox=None,
        legend_title=None,
        legend_ncol=1,
        category_labels=category_labels,
        show_grid=False,
    )

    # ==============================================================
    # SHARED LEGENDS
    # ==============================================================

    # Legend 1: pruning strategies (colors)
    legend_pruning = [
        mlines.Line2D([], [], color=colors[cat], linewidth=3, label=labels[cat])
        for cat in categories
    ]

    fig.legend(
        handles=handles_box[:4],
        labels = labels_box[:4],
        loc="upper left",
        bbox_to_anchor=(0.15, 0.99),
        ncol=2,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title="Pruning Strategies",
        title_fontproperties={
            "weight": "bold",
            "size": config.legend_title_fsize,
        },
    )

    # Legend 2: metrics (marker meaning) – uses black markers
    legend_stats = [
        mlines.Line2D(
            [], [], color="red", linestyle="--", label=f'Epoch size ({target_lines}s)'
        ),
        mlines.Line2D(
            [], [], color="black", linestyle="-",
            marker=marker_generated, label='# Generated labels'
        ),
        mlines.Line2D(
            [], [], color="black", linestyle=":",
            marker=marker_dominated, label='# Dominated labels'
        ),
    ]
    fig.legend(
        handles=legend_stats,
        loc="upper left",
        bbox_to_anchor=(0.66, 0.99),
        fontsize=config.legend_fsize,
        edgecolor="none",
        labelspacing=0.45,
        handlelength=2.6,
    )

    # Layout + save
    fig.tight_layout(rect=[0, 0, 1, 0.85])
    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path, bbox_inches="tight")
    plt.close(fig)

    return figure_path
