from dataclasses import dataclass


@dataclass
class PlotConfig:
    fig_size: tuple[float, float] = (6, 4)
    fig_size_half: tuple[float, float] = (6, 2.5)
    fig_size_small: tuple[float, float] = (4, 4)
    fig_size_wide: tuple[float, float] = (15, 4)
    axis_label_fsize: int = 10
    axis_title_fsize: int = 14
    axis_label_fsize_small: int = 9
    tick_label_fsize: int = 8
    tick_label_fsize_samll: int = 6
    legend_title_fsize: int = 9
    legend_fsize: int = 8
    legend_edgecolor: str = 'none'
    district_map_size: tuple[float, float] = (6, 4),
    map_legend_fsize: int = 8,
    region_fsize: int = 8,
    title_fsize = 9