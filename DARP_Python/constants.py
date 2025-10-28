plot_config = {
        "fig_size": (6, 4),
        "fig_size_half": (6, 2.5),
        "axis_label_fsize": 10,
        "tick_label_fsize": 8,
        "legend_title_fsize": 9,
        "legend_fsize": 8,
        "legend_edgecolor": 'none',
        "district_map_size": (7,7),
        "map_legend_fsize": 8,
        "region_fsize": 5,
}


# ---- MAP PARAMETERS ----
PLACE = "Manhattan, New York, USA"
MIN_STOP_SPACING_M = 120        # keep stops at least this far apart
HIGHWAY_BUFFER_M = 10           # how far from those roads to exclude
NETWORK_TYPE = "drive"          # use "drive" for pickup stops on roads

DATA_DIR = "Data"
TAXI_ZONE_DIR = "Data/taxi_zones"
RILEY_DIR = "Data/manhattan-network"
STOP_DIR = "Data/stops"
DAY_DIR = "Data/days"
TRANSFORM_DAY_DIR = "Data/transform_days/"

DATES_2015 = ['07-06', '07-15', '08-04', '08-28', '09-17', '09-26', '10-08',
              '10-25', '11-10', '11-30', '12-11', '12-30']
DATES_2016 = ['01-09', '01-29', '02-22', '02-25', '03-16', '03-29', '04-01',
              '04-27', '05-12', '05-21', '06-13', '06-28']