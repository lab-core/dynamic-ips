from matplotlib import pyplot as plt
from typing import Dict, List, Optional, Tuple, Any, Union, Callable
from Visualization.plot_config import PlotConfig
import seaborn as sns
from matplotlib.ticker import MultipleLocator


def plot_profile_line_group(
    ax: plt.Axes,
    df,
    x_column: str,
    y_column: str,
    group_column: Optional[str],
    groups: Optional[List[Any]],
    config: PlotConfig,
    palette: Optional[List[Any]] = None,
    group_labels: Optional[List[str]] = None,
    linewidth: float = 1.2,
    linestyle: str = "-",
    show_legend: bool = True,
    legend_loc: str = "best",
    legend_bbox: Optional[Tuple[float, float]] = None,
    legend_title: Optional[str] = None,
    legend_ncol: int = 1,
    xlabel: str = "",
    ylabel: str = "",
    show_xlabel: bool = True,
    show_ylabel: bool = True,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    x_major_step: Optional[float] = None,
    y_major_step: Optional[float] = None,
    grid: bool = True,
    target_lines: Optional[Union[float, Dict[str, float]]] = None,
) -> Tuple[List[Any], List[str]]:
    """
    Plot one or more lines on the given axis.
    No markers are used (lines only).

    target_lines:
        - None: no target line
        - float/int: single horizontal line at that y
        - dict[label -> value]: multiple horizontal lines (labels NOT used in legend)
    """

    # Basic axis styling
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if grid:
        ax.grid(True, color="lightgray", linestyle="--", alpha=0.5)
        ax.set_axisbelow(True)

    # Determine groups
    if group_column is None:
        groups = [None]
    else:
        if groups is None:
            groups = list(df[group_column].unique())

    n_groups = len(groups)

    # Colors
    if palette is None:
        palette = sns.color_palette("gist_earth", n_colors=n_groups)[::-1]

    if group_labels is None:
        group_labels = [str(g) if g is not None else y_column for g in groups]

    legend_handles = []
    legend_labels = []

    # Draw each line
    for i, g in enumerate(groups):
        sub_df = df if group_column is None else df[df[group_column] == g]

        if sub_df.empty:
            continue

        sub_df = sub_df.sort_values(by=x_column)

        line, = ax.plot(
            sub_df[x_column],
            sub_df[y_column],
            color=palette[i],
            linewidth=linewidth,
            linestyle=linestyle,
        )
        legend_handles.append(line)
        legend_labels.append(group_labels[i])

    # Add target lines (NOT included in legend)
    if target_lines is not None:
        if isinstance(target_lines, (int, float)):
            ax.axhline(
                y=target_lines,
                color="red",
                linestyle="--",
                linewidth=1.5,
            )
        elif isinstance(target_lines, dict):
            colors = ["red", "blue", "green", "orange", "purple"]
            for idx, (_, value) in enumerate(target_lines.items()):
                color = colors[idx % len(colors)]
                ax.axhline(
                    y=value,
                    color=color,
                    linestyle="--",
                    linewidth=1.5,
                )

    # Labels
    if show_ylabel and ylabel:
        ax.set_ylabel(ylabel, fontsize=config.axis_label_fsize, fontweight="bold")

    if show_xlabel and xlabel:
        ax.set_xlabel(xlabel, fontsize=config.axis_label_fsize, fontweight="bold")

    # Limits
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)

    # Tick formatting
    ax.tick_params(axis="both", which="major", labelsize=config.tick_label_fsize)

    if x_major_step is not None:
        ax.xaxis.set_major_locator(MultipleLocator(x_major_step))
    if y_major_step is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_major_step))

    # Legend
    if show_legend and legend_handles:
        ax.legend(
            legend_handles,
            legend_labels,
            loc=legend_loc,
            bbox_to_anchor=legend_bbox,
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_title,
            ncol=legend_ncol,
            title_fontproperties={
                "weight": "bold",
                "size": config.legend_title_fsize,
            },
        )

    return legend_handles, legend_labels



def create_multi_subplot_profile_lineplots(
    df,
    config: PlotConfig,
    subplot_configs: List[Dict[str, Any]],
    output_path: str,
    fig_size: Optional[Tuple[float, float]] = None,
    sharex: bool = False,
    sharey: bool = False,
    x_label_on_bottom_only: bool = True,
    figure_title: Optional[str] = None,
    tight_layout_rect: Optional[List[float]] = None,
    shared_legend: bool = False,
    shared_legend_config: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Create a vertically stacked series of line subplots.
    No markers are used in any subplot.

    shared_legend:
        If True, no legends are drawn on individual subplots.
        A single, shared legend is drawn on the figure using handles/labels
        from the first subplot.

    Each subplot_config may contain:
        - "target_lines": same semantics as in plot_profile_line_group
    """

    n_subplots = len(subplot_configs)

    if fig_size is None:
        base_w, base_h = config.fig_size
        fig_size = (base_w, base_h * n_subplots / 2.0)

    fig, axes = plt.subplots(
        n_subplots,
        1,
        figsize=fig_size,
        sharex=sharex,
        sharey=sharey
    )

    if n_subplots == 1:
        axes = [axes]

    all_legend_handles: List[Any] = []
    all_legend_labels: List[str] = []

    for idx, subplot_cfg in enumerate(subplot_configs):
        ax = axes[idx]

        df_subplot = df.copy()
        if subplot_cfg.get("filter") is not None:
            df_subplot = subplot_cfg["filter"](df_subplot)

        # Extract config
        x_column = subplot_cfg["x_column"]
        y_column = subplot_cfg["y_column"]
        group_column = subplot_cfg.get("group_column")
        groups = subplot_cfg.get("groups")
        group_labels = subplot_cfg.get("group_labels")
        palette = subplot_cfg.get("palette")

        ylabel = subplot_cfg.get("ylabel", y_column)
        xlabel = subplot_cfg.get("xlabel", "")

        show_xlabel = subplot_cfg.get("show_xlabel", True)
        if sharex and x_label_on_bottom_only and idx < n_subplots - 1:
            ax.tick_params(labelbottom=False)
            show_xlabel = False

        # Per-subplot target lines
        target_lines = subplot_cfg.get("target_lines", None)

        # Per-subplot legend control
        subplot_show_legend = subplot_cfg.get("show_legend", True)
        if shared_legend:
            # Suppress individual legends when using a shared legend
            subplot_show_legend = False

        handles, labels = plot_profile_line_group(
            ax=ax,
            df=df_subplot,
            x_column=x_column,
            y_column=y_column,
            group_column=group_column,
            groups=groups,
            config=config,
            palette=palette,
            group_labels=group_labels,
            xlabel=xlabel,
            ylabel=ylabel,
            show_xlabel=show_xlabel,
            show_ylabel=subplot_cfg.get("show_ylabel", True),
            xlim=subplot_cfg.get("xlim"),
            ylim=subplot_cfg.get("ylim"),
            x_major_step=subplot_cfg.get("x_major_step"),
            y_major_step=subplot_cfg.get("y_major_step"),
            linewidth=subplot_cfg.get("linewidth", 1),
            linestyle=subplot_cfg.get("linestyle", "-"),
            grid=subplot_cfg.get("grid", True),
            show_legend=subplot_show_legend,
            legend_loc=subplot_cfg.get("legend_config", {}).get("loc", "best"),
            legend_bbox=subplot_cfg.get("legend_config", {}).get("bbox"),
            legend_title=subplot_cfg.get("legend_config", {}).get("title"),
            legend_ncol=subplot_cfg.get("legend_config", {}).get("ncol", 1),
            target_lines=target_lines,
        )

        # Collect handles/labels for shared legend (use the first subplot)
        if shared_legend and idx == 0:
            all_legend_handles = handles
            all_legend_labels = labels

        if subplot_cfg.get("title"):
            ax.set_title(
                subplot_cfg["title"],
                fontsize=config.title_fsize,
                fontweight="bold",
            )

    # Shared legend at figure level
    if shared_legend and all_legend_handles:
        legend_cfg = shared_legend_config or {}
        fig.legend(
            all_legend_handles,
            all_legend_labels,
            loc=legend_cfg.get("loc", "upper center"),
            bbox_to_anchor=legend_cfg.get("bbox", (0.5, 0.98)),
            ncol=legend_cfg.get("ncol", len(all_legend_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_cfg.get("title"),
            title_fontproperties={
                "weight": "bold",
                "size": config.legend_title_fsize,
            },
            framealpha=legend_cfg.get("framealpha", 1.0),
            facecolor=legend_cfg.get("facecolor", "white"),
        )

    if figure_title:
        fig.suptitle(
            figure_title,
            fontsize=config.title_fsize + 2,
            fontweight="bold",
            y=1.02,
        )

    if tight_layout_rect:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()

    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path


def create_multi_subplot_profile_lineplots(
    df,
    config: PlotConfig,
    subplot_configs: List[Dict[str, Any]],
    output_path: str,
    fig_size: Optional[Tuple[float, float]] = None,
    sharex: bool = False,
    sharey: bool = False,
    x_label_on_bottom_only: bool = True,
    figure_title: Optional[str] = None,
    tight_layout_rect: Optional[List[float]] = None,
    shared_legend: bool = False,
    shared_legend_config: Optional[Dict[str, Any]] = None,
    grid_shape: Optional[Tuple[int, int]] = None,   # <<< NEW
) -> str:
    """
    Create a series of line subplots.
    If grid_shape is None, subplots are stacked vertically as before.
    Otherwise a grid of (n_rows, n_cols) is used.

    shared_legend:
        If True, no legends are drawn on individual subplots.
        A single, shared legend is drawn on the figure using handles/labels
        from the first subplot.
    """

    n_subplots = len(subplot_configs)

    # Determine grid shape
    if grid_shape is not None:
        n_rows, n_cols = grid_shape
        if n_rows * n_cols != n_subplots:
            raise ValueError(
                f"grid_shape {grid_shape} does not match number of subplots ({n_subplots})"
            )
    else:
        n_rows, n_cols = n_subplots, 1

    # Figure size
    if fig_size is None:
        base_w, base_h = config.fig_size
        # simple scaling with number of rows/cols
        fig_size = (base_w * n_cols, base_h * n_rows / 2.0)

    fig, axes = plt.subplots(
        n_rows,
        n_cols,
        figsize=fig_size,
        sharex=sharex,
        sharey=sharey
    )

    # Normalize axes to a 1D list for easy indexing
    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    all_legend_handles: List[Any] = []
    all_legend_labels: List[str] = []

    for idx, subplot_cfg in enumerate(subplot_configs):
        ax = axes[idx]

        df_subplot = df.copy()
        if subplot_cfg.get("filter") is not None:
            df_subplot = subplot_cfg["filter"](df_subplot)

        # Extract config
        x_column = subplot_cfg["x_column"]
        y_column = subplot_cfg["y_column"]
        group_column = subplot_cfg.get("group_column")
        groups = subplot_cfg.get("groups")
        group_labels = subplot_cfg.get("group_labels")
        palette = subplot_cfg.get("palette")

        ylabel = subplot_cfg.get("ylabel", y_column)
        xlabel = subplot_cfg.get("xlabel", "")

        show_xlabel = subplot_cfg.get("show_xlabel", True)

        # Handle x labels only on bottom row when shared x-axis
        if sharex and x_label_on_bottom_only:
            if grid_shape is None:
                # old behavior: only last subplot gets labels
                if idx < n_subplots - 1:
                    ax.tick_params(labelbottom=False)
                    show_xlabel = False
            else:
                # grid: only subplots in the last row get x labels
                row = idx // n_cols
                if row < n_rows - 1:
                    ax.tick_params(labelbottom=False)
                    show_xlabel = False

        # Per-subplot target lines
        target_lines = subplot_cfg.get("target_lines", None)

        # Per-subplot legend control
        subplot_show_legend = subplot_cfg.get("show_legend", True)
        if shared_legend:
            # Suppress individual legends when using a shared legend
            subplot_show_legend = False

        handles, labels = plot_profile_line_group(
            ax=ax,
            df=df_subplot,
            x_column=x_column,
            y_column=y_column,
            group_column=group_column,
            groups=groups,
            config=config,
            palette=palette,
            group_labels=group_labels,
            xlabel=xlabel,
            ylabel=ylabel,
            show_xlabel=show_xlabel,
            show_ylabel=subplot_cfg.get("show_ylabel", True),
            xlim=subplot_cfg.get("xlim"),
            ylim=subplot_cfg.get("ylim"),
            x_major_step=subplot_cfg.get("x_major_step"),
            y_major_step=subplot_cfg.get("y_major_step"),
            linewidth=subplot_cfg.get("linewidth", 1),
            linestyle=subplot_cfg.get("linestyle", "-"),
            grid=subplot_cfg.get("grid", True),
            show_legend=subplot_show_legend,
            legend_loc=subplot_cfg.get("legend_config", {}).get("loc", "best"),
            legend_bbox=subplot_cfg.get("legend_config", {}).get("bbox"),
            legend_title=subplot_cfg.get("legend_config", {}).get("title"),
            legend_ncol=subplot_cfg.get("legend_config", {}).get("ncol", 1),
            target_lines=target_lines,
        )

        # Collect handles/labels for shared legend (use the first subplot)
        if shared_legend and idx == 0:
            all_legend_handles = handles
            all_legend_labels = labels

        if subplot_cfg.get("title"):
            ax.set_title(
                subplot_cfg["title"],
                fontsize=config.title_fsize,
                fontweight="bold",
            )

    # Shared legend at figure level
    if shared_legend and all_legend_handles:
        legend_cfg = shared_legend_config or {}
        fig.legend(
            all_legend_handles,
            all_legend_labels,
            loc=legend_cfg.get("loc", "upper center"),
            bbox_to_anchor=legend_cfg.get("bbox", (0.5, 0.98)),
            ncol=legend_cfg.get("ncol", len(all_legend_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_cfg.get("title"),
            title_fontproperties={
                "weight": "bold",
                "size": config.legend_title_fsize,
            },
            framealpha=legend_cfg.get("framealpha", 1.0),
            facecolor=legend_cfg.get("facecolor", "white"),
        )

    if figure_title:
        fig.suptitle(
            figure_title,
            fontsize=config.title_fsize + 2,
            fontweight="bold",
            y=1.02,
        )

    if tight_layout_rect:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()

    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    return output_path
