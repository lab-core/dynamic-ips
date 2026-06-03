import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import seaborn as sns
import os
from typing import Dict, List, Optional, Tuple, Any, Callable

from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.plot_config import PlotConfig
from Visualization.visualize_box_plots import calculate_positions


# ----------------------------------------------------------------------
# 1. Data prep for violins (per item × category)
# ----------------------------------------------------------------------

def make_half_violin(pc, side="left"):
    """
    Modify a PolyCollection from ax.violinplot so that only
    the left or right half of the violin remains.
    """
    path = pc.get_paths()[0]
    verts = path.vertices
    xs = verts[:, 0]
    xmid = xs.mean()

    if side == "left":
        # clamp everything to the right of the center
        verts[xs > xmid, 0] = xmid
    elif side == "right":
        # clamp everything to the left of the center
        verts[xs < xmid, 0] = xmid


def prepare_violin_data(
    df,
    items,
    items_column: str,
    categories: List[Any],
    category_column: str,
    value_column: str,
    compute_zero_pct: bool = False,
    zero_tol: float = 1e-10,
):
    """
    Prepare data for violin visualization.

    Parameters
    ----------
    df : DataFrame
        Filtered dataframe.
    items : array-like
        Ordered list of x-axis items.
    items_column : str
        Column name for x-axis items.
    categories : list
        Category values to plot.
    category_column : str
        Column name for categories.
    value_column : str
        Column name for values.
    compute_zero_pct : bool
        If True, also compute percentage of "zero" values per group.
    zero_tol : float
        Tolerance for |value| < zero_tol to be counted as zero.

    Returns
    -------
    plot_data : dict
        {category: [array(values for item_0), ..., array(values for item_n)]}
    zero_pct_data : dict or None
        Same keys/structure as plot_data, but with percentages instead of arrays,
        or None if compute_zero_pct == False.
    """
    plot_data: Dict[Any, List[np.ndarray]] = {cat: [] for cat in categories}
    zero_pct_data: Optional[Dict[Any, List[float]]] = None
    if compute_zero_pct:
        zero_pct_data = {cat: [] for cat in categories}

    for item in items:
        item_mask = df[items_column] == item
        for cat in categories:
            cat_mask = df[category_column].str.strip() == str(cat)
            vals = df[item_mask & cat_mask][value_column].dropna().values

            plot_data[cat].append(np.asarray(vals, dtype=float))

            if compute_zero_pct and zero_pct_data is not None:
                if vals.size == 0:
                    zero_pct_data[cat].append(0.0)
                else:
                    zp = 100.0 * np.mean(np.abs(vals) < zero_tol)
                    zero_pct_data[cat].append(float(zp))

    return plot_data, zero_pct_data


# ----------------------------------------------------------------------
# 2. Grouped violin plotting on a single axis
# ----------------------------------------------------------------------
def plot_violin_group(
    ax: plt.Axes,
    plot_data: Dict[Any, List[np.ndarray]],
    categories: List[Any],
    palette,
    config: PlotConfig,
    show_ylabel: bool = True,
    ylabel: str = "Value",
    width: float = 0.5,
    gap_factors: Optional[List[float]] = None,
    target_lines: Optional[Any] = None,
    xlabel: str = "",
    x_tick_labels: Optional[List[str]] = None,
    rotation: int = 15,
    ylim: Optional[Tuple[float, float]] = None,
    show_category_legend: bool = True,
    legend_loc: str = "best",
    legend_bbox: Optional[Tuple[float, float]] = None,
    legend_title: Optional[str] = None,
    legend_ncol: int = 1,
    category_labels: Optional[List[str]] = None,
    show_grid: bool = False,
    ylabel_pad: Optional[float] = None,
    zero_pct_data: Optional[Dict[Any, List[float]]] = None,
    zero_y_offset: float = -0.012,
) -> Tuple[List[Any], List[str], List[Any], List[str]]:
    """
    Create grouped violins on the given axis.

    * Uses Matplotlib's ax.violinplot
    * Adds custom 5th / 95th percentile lines
    * Optionally adds zero-percentage text for each violin
    * Returns BOTH category legend handles and statistic legend handles.

    Parameters
    ----------
    ax : Axes
        Axis to plot on.
    plot_data : dict
        {category: list of 1D arrays (one per item)}.
    categories : list
        Category names (keys of plot_data).
    palette : list
        Colors for categories.
    config : PlotConfig
        Styling configuration.
    [many kwargs mirror plot_boxplot_group]

    Returns
    -------
    category_handles, category_labels, stats_handles, stats_labels
    """
    # Basic axis styling
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if show_grid:
        ax.grid(True, alpha=0.3, linestyle="--", axis="y")
        ax.set_axisbelow(True)

    # Colors
    darkened_palette = [darken_color(c, factor=0.8) for c in palette]
    light_palette = [(r, g, b, 0.5) for (r, g, b) in palette]

    # Colors used for stats (same across categories, like your original)
    median_color = darkened_palette[0]
    mean_color = darkened_palette[-1]
    dist_facecolor = light_palette[0]
    dist_edgecolor = darkened_palette[0]

    # Positions
    n_items = len(next(iter(plot_data.values())))
    n_categories = len(categories)
    base_positions, positions = calculate_positions(
        n_items, n_categories, width=width, gap_factors=gap_factors
    )

    # Category legend
    if category_labels is None:
        category_labels = [str(c) for c in categories]
    category_handles: List[Any] = []

    # Plot each category's violins
    for i, cat in enumerate(categories):
        data_list = plot_data[cat]

        # If absolutely no data for this category, skip
        if all(len(arr) == 0 for arr in data_list):
            continue

        pos = positions[i]

        data_list_full = plot_data[cat]
        data_list_plot = []
        if i<2:
            for arr in data_list_full:
                a = np.asarray(arr, dtype=float)
                if a.size == 0:
                    data_list_plot.append(a)
                    continue

                p_lo = np.percentile(a, 0)
                p_hi = np.percentile(a, 99.8)
                trimmed = a[(a >= p_lo) & (a <= p_hi)]

                # KDE needs enough points; fall back safely
                if trimmed.size < 2:
                    trimmed = a

                data_list_plot.append(trimmed)
        else:
            data_list_plot = data_list_full



        parts = ax.violinplot(
            data_list_plot,
  #          data_list,
            positions=pos,
            widths=width,
            showmeans=True,
            showmedians=True,
            showextrema=False,
        )

        # Violin bodies
        for pc in parts["bodies"]:
            pc.set_facecolor(light_palette[i])
            pc.set_edgecolor(darkened_palette[i])
            pc.set_alpha(0.6)

        # Means / medians use global stats colors
        parts["cmeans"].set_color(mean_color)
        parts["cmeans"].set_linewidth(2)
        parts["cmedians"].set_color(median_color)
        parts["cmedians"].set_linewidth(1.5)

        # Keep first body as handle for category legend
        category_handles.append(parts["bodies"][0])

        # Per-violin 5–95% lines + optional zero text
        zp_list = None
        if zero_pct_data is not None:
            zp_list = zero_pct_data.get(cat, None)

        for x_pos, vals_idx in zip(pos, range(len(data_list))):
            vals = np.asarray(data_list[vals_idx], dtype=float)
            if vals.size == 0:
                continue

            p5 = np.percentile(vals, 5)
            p95 = np.percentile(vals, 95)

            ax.plot(
                [x_pos, x_pos],
                [p5, p95],
                color="black",
                linestyle=":",
                linewidth=1,
                alpha=0.8,
            )

            if zp_list is not None and vals_idx < len(zp_list):
                zp = zp_list[vals_idx]
                if zp > 0:
                    ax.text(
                        x_pos,
                        zero_y_offset,
                        f"{zp:.1f}%",
                        ha="center",
                        va="bottom",
                        fontsize=6,
                        fontweight="bold",
                    )

    # Labels
    if show_ylabel:
        if ylabel_pad is not None:
            ax.set_ylabel(
                ylabel,
                fontsize=config.axis_label_fsize,
                fontweight="bold",
                labelpad=ylabel_pad,
            )
        else:
            ax.set_ylabel(
                ylabel,
                fontsize=config.axis_label_fsize,
                fontweight="bold",
            )

    if xlabel:
        ax.set_xlabel(
            xlabel,
            fontsize=config.axis_label_fsize_small,
            fontweight="bold",
        )

    # X ticks
    if x_tick_labels is not None:
        ax.set_xticks(base_positions)
        ax.set_xticklabels(
            x_tick_labels,
            rotation=rotation,
            fontsize=config.tick_label_fsize,
        )

    # Tick config
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.tick_params(which="minor", length=0)
    ax.tick_params(
        axis="both",
        which="major",
        labelsize=config.tick_label_fsize,
        length=3.8,
    )

    # Y limits
    if ylim is not None:
        if isinstance(ylim, (int, float)):
            ax.set_ylim(top=ylim)
        else:
            ax.set_ylim(ylim)

 #   ax.set_yticks(np.arange(0, 0.24, 0.05))
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

    # Optional target lines (same as in boxplot helper)
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
            for idx, (label, value) in enumerate(target_lines.items()):
                color = colors[idx % len(colors)]
                ax.axhline(
                    y=value,
                    color=color,
                    linestyle="--",
                    linewidth=1.5,
                    label=label,
                )

    # Category legend (colors) – like boxplots
    if show_category_legend and category_handles:
        if legend_bbox:
            ax.legend(
                category_handles,
                category_labels,
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
        else:
            ax.legend(
                category_handles,
                category_labels,
                loc=legend_loc,
                fontsize=config.legend_fsize,
                edgecolor=config.legend_edgecolor,
                title=legend_title,
                ncol=legend_ncol,
                title_fontproperties={
                    "weight": "bold",
                    "size": config.legend_title_fsize,
                },
            )

    # Stats legend elements (same style as your prior violin function)
    stats_handles = [
        mlines.Line2D(
            [0],
            [0],
            color=median_color,
            linewidth=2,
            label="Median",
        ),
        mlines.Line2D(
            [0],
            [0],
            color=mean_color,
            linewidth=2,
            label="Mean",
        ),
        mlines.Line2D(
            [0],
            [0],
            color="black",
            linestyle=":",
            linewidth=1,
            alpha=0.8,
            label="5–95 %",
        ),
 #       plt.Rectangle( (0, 0), 1, 1, facecolor=dist_facecolor,
 #           edgecolor=dist_edgecolor, alpha=0.7, label="Distribution"),
    ]
    stats_labels = [h.get_label() for h in stats_handles]

    return category_handles, category_labels, stats_handles, stats_labels


# ----------------------------------------------------------------------
# 3. Multi-subplot factory for violins (boxplot analogue)
# ----------------------------------------------------------------------
def create_multi_subplot_violinplots(
    data_path: str,
    config: PlotConfig,
    subplot_configs: List[Dict[str, Any]],
    output_filename: str,
    fig_size: Optional[Tuple[float, float]] = None,
    n_rows: Optional[int] = None,
    n_cols: Optional[int] = None,
    shared_legend: bool = False,
    shared_legend_config: Optional[Dict[str, Any]] = None,
    shared_stats_legend: bool = False,
    shared_stats_legend_config: Optional[Dict[str, Any]] = None,
    figure_title: Optional[str] = None,
    tight_layout_rect: Optional[List[float]] = None,
    additional_filter: Optional[Callable] = None,
    color_reverse: bool = False,
    palette_name: str = "gist_earth",
    palette=None,
) -> str:
    """
    Create multiple grouped-violin subplots with maximum flexibility.

    Very close in spirit to create_multi_subplot_boxplots, but calling
    plot_violin_group and including your custom
    5th/95th-percentile + zero-percentage text + stats legend style.

    Each subplot_config accepts (same as boxplots, plus a few extras):

      - 'item_column'
      - 'category_column'
      - 'value_column'
      - 'categories'
      - 'category_labels'
      - 'ylabel'
      - 'xlabel'
      - 'title'
      - 'filter'
      - 'ylim'
      - 'target_lines'
      - 'show_ylabel'
      - 'show_legend'          (category legend)
      - 'width'
      - 'rotation'
      - 'gap_factors'
      - 'show_grid'
      - 'ylabel_pad'
      - 'data_path' / 'data_df'  (optional, like in boxplots)
      - 'show_zero_text' (bool, default False)
      - 'zero_tol'       (float, default 1e-10)
      - 'zero_y_offset'  (float, default -0.012)

    Parameters mirror create_multi_subplot_boxplots, with two additions:
      - shared_stats_legend: bool
      - shared_stats_legend_config: dict

    Returns
    -------
    figure_path : str
        Full path to saved PNG.
    """
    # Read global data
    df_global = read_csv_with_encoding(data_path)
    if additional_filter is not None:
        df_global = additional_filter(df_global)

    # Cache for per-subplot CSVs
    df_cache = {data_path: df_global}

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
        fig_size = (3 * n_cols, 4 * n_rows)

    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size)

    # Make axes iterable
    if n_subplots == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if (n_rows > 1 or n_cols > 1) else [axes]

    # Palette
    max_categories = max(len(cfg["categories"]) for cfg in subplot_configs)
    if palette is None:
        palette = sns.color_palette(palette_name, n_colors=max_categories)
    if color_reverse:
        palette = palette[::-1]

    all_cat_handles: List[Any] = []
    all_cat_labels: List[str] = []
    all_stats_handles: List[Any] = []
    all_stats_labels: List[str] = []

    # Build each subplot
    for idx, subplot_config in enumerate(subplot_configs):
        ax = axes[idx]

        # Core config
        item_column = subplot_config.get("item_column", "Instance")
        x_tick_labels = subplot_config.get("x_tick_labels", None)
        category_column = subplot_config["category_column"]
        value_column = subplot_config["value_column"]
        categories = subplot_config["categories"]
        category_labels = subplot_config.get("category_labels", categories)
        ylabel = subplot_config.get("ylabel", value_column)
        xlabel = subplot_config.get("xlabel", "")
        subplot_title = subplot_config.get("title", "")
        subplot_filter = subplot_config.get("filter", None)
        ylim = subplot_config.get("ylim", None)
        target_lines = subplot_config.get("target_lines", None)
        show_ylabel = subplot_config.get("show_ylabel", True)
        show_legend = subplot_config.get("show_legend", not shared_legend)
        width = subplot_config.get("width", 0.5)
        rotation = subplot_config.get("rotation", 15)
        legend_config = subplot_config.get("legend_config", {})
        gap_factors = subplot_config.get("gap_factors", None)
        show_grid = subplot_config.get("show_grid", False)
        ylabel_pad = subplot_config.get("ylabel_pad", None)

        show_zero_text = subplot_config.get("show_zero_text", False)
        zero_tol = subplot_config.get("zero_tol", 1e-10)
        zero_y_offset = subplot_config.get("zero_y_offset", -0.012)

        # Decide dataframe for this subplot
        subplot_data_path = subplot_config.get("data_path", None)
        subplot_df = subplot_config.get("data_df", None)

        if subplot_df is not None:
            df_subplot = subplot_df.copy()
        else:
            if subplot_data_path is not None:
                if subplot_data_path in df_cache:
                    df_subplot = df_cache[subplot_data_path].copy()
                else:
                    df_tmp = read_csv_with_encoding(subplot_data_path)
                    if additional_filter is not None:
                        df_tmp = additional_filter(df_tmp)
                    df_cache[subplot_data_path] = df_tmp
                    df_subplot = df_tmp.copy()
            else:
                df_subplot = df_global.copy()

        # Apply subplot-specific filter
        if subplot_filter is not None:
            df_subplot = subplot_filter(df_subplot)

        # Ensure string columns for consistency
        df_subplot[item_column] = df_subplot[item_column].astype(str)
        if category_column is None:
            category_column = "_dummy_category"
            df_subplot[category_column] = str(categories[0])
        else:
            df_subplot[category_column] = df_subplot[category_column].astype(str)

        # Determine x-items (sorted)
        x_items = np.sort(df_subplot[item_column].unique()).astype(str)

        # Prepare violin data
        plot_data, zero_pct_data = prepare_violin_data(
            df_subplot,
            x_items,
            item_column,
            categories,
            category_column,
            value_column,
            compute_zero_pct=show_zero_text,
            zero_tol=zero_tol,
        )

        subplot_palette = palette[: len(categories)]

        if x_tick_labels is None:
            x_tick_labels = x_items

        # Plot violins
        cat_handles, cat_labels, stats_handles, stats_labels = plot_violin_group(
            ax=ax,
            plot_data=plot_data,
            categories=categories,
            palette=subplot_palette,
            config=config,
            show_ylabel=show_ylabel,
            ylabel=ylabel,
            width=width,
            gap_factors=gap_factors,
            target_lines=target_lines,
            xlabel=xlabel,
            x_tick_labels=x_tick_labels,
            rotation=rotation,
            ylim=ylim,
            show_category_legend=show_legend,
            legend_loc=legend_config.get("loc", "best"),
            legend_bbox=legend_config.get("bbox", None),
            legend_title=legend_config.get("title", None),
            legend_ncol=legend_config.get("ncol", 1),
            category_labels=category_labels,
            show_grid=show_grid,
            ylabel_pad=ylabel_pad,
            zero_pct_data=zero_pct_data if show_zero_text else None,
            zero_y_offset=zero_y_offset,
        )

        # Subplot title
        if subplot_title:
            ax.set_title(
                subplot_title,
                fontsize=config.title_fsize,
                fontweight="bold",
            )

        # Collect handles for shared legends (first subplot)
        if idx == 0:
            all_cat_handles = cat_handles
            all_cat_labels = cat_labels
            all_stats_handles = stats_handles
            all_stats_labels = stats_labels

    # Hide any unused axes
    for idx in range(n_subplots, len(axes)):
        axes[idx].set_visible(False)

    # Shared category legend (colors)
    if shared_legend and all_cat_handles:
        legend_config = shared_legend_config or {}
        fig.legend(
            handles=all_cat_handles,
            labels=all_cat_labels,
            loc=legend_config.get("loc", "upper center"),
            bbox_to_anchor=legend_config.get("bbox", (0.5, 0.98)),
            ncol=legend_config.get("ncol", len(all_cat_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            title=legend_config.get("title", None),
            title_fontproperties={
                "weight": "bold",
                "size": config.legend_title_fsize,
            },
            framealpha=legend_config.get("framealpha", 1.0),
            facecolor=legend_config.get("facecolor", "white"),
        )

    # Shared stats legend (mean/median/5–95%/distribution)
    if shared_stats_legend and all_stats_handles:
        stats_cfg = shared_stats_legend_config or {}
        fig.legend(
            handles=all_stats_handles,
            labels=all_stats_labels,
            loc=stats_cfg.get("loc", "upper right"),
            bbox_to_anchor=stats_cfg.get("bbox", (0.98, 0.98)),
            ncol=stats_cfg.get("ncol", len(all_stats_labels)),
            fontsize=config.legend_fsize,
            edgecolor=config.legend_edgecolor,
            framealpha=stats_cfg.get("framealpha", 1.0),
            facecolor=stats_cfg.get("facecolor", "white"),
        )

    # Figure title
    if figure_title:
        fig.suptitle(
            figure_title,
            fontsize=config.title_fsize + 2,
            fontweight="bold",
            y=1.02,
        )

    # Layout
    if tight_layout_rect:
        fig.tight_layout(rect=tight_layout_rect)
    else:
        fig.tight_layout()

    figure_path = os.path.join(os.path.dirname(data_path), output_filename)
    fig.savefig(figure_path)
    plt.close(fig)
    return figure_path


