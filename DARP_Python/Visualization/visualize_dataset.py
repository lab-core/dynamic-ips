import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import constants as c
import os
from matplotlib import gridspec
from matplotlib.patches import ConnectionPatch
import math

from Visualization.visualize_network import plot_districts


def plot_requests_arrival(df_dataset, parent_folder, time_origin):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day

    df = df_dataset.copy()  # Safe local copy

    df['count'] = 1
    df['tpep_pickup_datetime'] = pd.to_datetime(df['tpep_pickup_datetime'])

    # Group data into 30-second intervals
    df_grouped = df.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='30s')).sum(numeric_only=True)

    # Extract hours and minutes from the index
    hour_list = [d.strftime('%H:%M') for d in df_grouped.index]

    fig, ax = plt.subplots(figsize=c.plot_config["fig_size_half"])

    num_requests = df_grouped['count'].tolist()
    ax.plot(hour_list, num_requests, color='yellowgreen')
    vertical_lines = ['07:00', '23:00']

    # Shade the areas between 07:00-09:00 and 12:00-14:00
    ax.axvspan('07:00', '23:00', color='lightgray', alpha=0.3)

    for time in vertical_lines:
        ax.axvline(x=time, color='gray', linewidth=0.5, linestyle='--')

    # Calculate the average number of requests and plot the horizontal line
    average_requests = np.mean(num_requests)
    ax.axhline(y=average_requests, color='darkslategray', label=f'Average: {average_requests:.1f}', linewidth=1)


    limit = 24
    hour_list.append('24:00')
    hour_list.append('24:00')
    num_ticks = len(hour_list) // limit  # Assuming 24 hours
    x_ticks = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    x_labels = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    # Add the last time point to the x-ticks and labels
    x_ticks.append(hour_list[-1])
    x_labels.append(hour_list[-1])
    plt.xticks(rotation=40)
    plt.tick_params(axis='both', labelsize=c.plot_config["tick_label_fsize"])
    plt.xticks(x_ticks, x_labels)
    ax.set_ylabel('Number of requests', fontsize=c.plot_config["axis_label_fsize"], fontweight='bold')
    ax.grid(axis='x', linestyle='--', color='lightgray')
    ax.legend(loc='lower right', fontsize=c.plot_config["legend_fsize"], edgecolor=c.plot_config["legend_edgecolor"],
              framealpha=1.0, facecolor='white')

    ax.set_xlim(x_ticks[0], x_ticks[-1])
    ax.set_ylim(0, 252)


    # Adjust layout
    plt.tight_layout()

    image_dir = parent_folder + "Request_Arrival/"
    file_name = f"{limit}_{year}{month}{day}_requests.png"
    os.makedirs(image_dir, exist_ok=True)
    plt.savefig(image_dir + file_name, dpi=300, bbox_inches="tight")

    plt.close(fig)
    return average_requests

def plot_zoom_request_arrival_4h(df_dataset, parent_folder, time_origin):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day

    df = df_dataset.copy()  # Safe local copy

    df['count'] = 1
    df['tpep_pickup_datetime'] = pd.to_datetime(df['tpep_pickup_datetime'])

    # Group data into 30-second intervals
    df_grouped = df.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='30s')).sum(numeric_only=True)

    # Extract hours and minutes from the index
    hour_list = [d.strftime('%H:%M') for d in df_grouped.index]

    # Create figure and define column widths (top left 2h gets less width)
    fig = plt.figure(figsize=c.plot_config["fig_size"])
    gs = gridspec.GridSpec(2, 2, width_ratios=[1, 2])  # 1:2 ratio for 2h vs 4h

    # Top zoomed plots

    # Main plot spans bottom row (two columns)
    plot1 = fig.add_subplot(gs[1, :])  # main plot


    # Main plot
#    plot1 = fig.add_subplot(2, 2, (3,4))
    num_requests = df_grouped['count'].tolist()
    plot1.plot(hour_list, num_requests, color='yellowgreen')
    vertical_lines = ['07:00', '09:00','10:00','11:00', '15:00']

    # Shade the areas between 07:00-09:00 and 12:00-14:00
    plot1.axvspan('07:00', '09:00', color='darkseagreen', alpha=0.3)
    plot1.axvspan('11:00', '15:00', color='lightblue', alpha=0.3)
    plot1.axvspan('10:00', '11:00', color='khaki', alpha=0.5)

    for time in vertical_lines:
        plot1.axvline(x=time, color='gray', linewidth=0.5, linestyle='--')

    # Calculate the average number of requests and plot the horizontal line
    average_requests = np.mean(num_requests)
    plot1.axhline(y=average_requests, color='darkslategray', label=f'Average: {average_requests:.1f}', linewidth=1)


    limit = 24
    hour_list.append('24:00')
    hour_list.append('24:00')
    num_ticks = len(hour_list) // limit  # Assuming 24 hours
    x_ticks = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    x_labels = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    # Add the last time point to the x-ticks and labels
    x_ticks.append(hour_list[-1])
    x_labels.append(hour_list[-1])
    plt.xticks(rotation=40)
    plt.tick_params(axis='both', labelsize=c.plot_config["tick_label_fsize"])
    plt.xticks(x_ticks, x_labels)
    plot1.set_ylabel('Number of requests', fontsize=c.plot_config["axis_label_fsize"], fontweight='bold')
    plot1.grid(axis='x', linestyle='--', color='lightgray')
    plot1.legend(fontsize=7, loc='lower right', facecolor='white')

    plot1.set_xlim(x_ticks[0], x_ticks[-1])
    plot1.set_ylim(0, 240)

    # Create zoomed subplots for the highlighted regions
#    plot2 = fig.add_subplot(2, 2, 1)
    plot2 = fig.add_subplot(gs[0, 0])  # 2-hour zoom (narrower)
    hour_list2 = []
    num_requests2 = []

    for i, time in enumerate(hour_list):
        if '07:00' <= time <= '09:00':
            hour_list2.append(time)
            num_requests2.append(df_grouped['count'].tolist()[i])
    plot2.plot(hour_list2, num_requests2, color='darkcyan')
    average_requests2 = np.mean(num_requests2)
    plot2.axhline(y=average_requests2, color='darkslategray', label=f'Average: {average_requests2:.1f}', linewidth=1)

    num_ticks2 = len(hour_list2) // 2
    x_ticks2 = [hour_list2[i] for i in range(0, len(hour_list2), num_ticks2)]
    x_labels2 = [hour_list2[i] for i in range(0, len(hour_list2), num_ticks2)]
    x_ticks2.append(hour_list2[-1])
    x_labels2.append(hour_list2[-1])
    plt.tick_params(axis='both', labelsize=c.plot_config["tick_label_fsize"])
    plt.xticks(x_ticks2, x_labels2)
    plot2.set_ylabel('Number of requests', fontsize=c.plot_config["axis_label_fsize"], fontweight='bold')
    plt.grid(axis='x', linestyle='--', color='lightgray')
    plot2.legend(fontsize=7,  facecolor='white')
    plot2.set_xlim('07:00', '09:00')
    plot2.set_ylim(0, 240)

#    plot3 = fig.add_subplot(2, 2, 2)
    plot3 = fig.add_subplot(gs[0, 1])  # 4-hour zoom (wider)
    hour_list3 = []
    num_requests3 = []

    for i, time in enumerate(hour_list):
        if '11:00' <= time <= '15:00':
            hour_list3.append(time)
            num_requests3.append(df_grouped['count'].tolist()[i])
    plot3.plot(hour_list3, num_requests3, color='darkcyan')
    average_requests3 = np.mean(num_requests3)
    plot3.axhline(y=average_requests3, color='darkslategray', label=f'Average: {average_requests3:.1f}', linewidth=1)

    num_ticks3 = len(hour_list3) // 4
    x_ticks3 = [hour_list3[i] for i in range(0, len(hour_list3), num_ticks3)]
    x_labels3 = [hour_list3[i] for i in range(0, len(hour_list3), num_ticks3)]
    x_ticks3.append(hour_list3[-1])
    x_labels3.append(hour_list3[-1])
    plt.tick_params(axis='both', labelsize=c.plot_config["tick_label_fsize"])
    plt.xticks(x_ticks3, x_labels3)
#    plot3.set_ylabel('Number of requests', fontsize=text_size, fontweight='bold')
    plt.grid(axis='x', linestyle='--', color='lightgray')
    plot3.set_xlim('11:00', '15:00')
    plot3.legend(fontsize=7,  facecolor='white')
    plot3.set_ylim(0, 240)

    # Connection lines between the main plot and zoomed plots
    conn1 = ConnectionPatch(xyA=(hour_list.index('07:00')/2, num_requests[int(hour_list.index('07:00'))]), coordsA=plot1.transData,
                            xyB=(hour_list2.index('07:00')/2, num_requests2[int(hour_list2.index('07:00'))]), coordsB=plot2.transData, color='gray', linewidth=0.5, linestyle='--')
    fig.add_artist(conn1)
    conn2 = ConnectionPatch(xyA=(hour_list.index('09:00')/2, num_requests[int(hour_list.index('09:00'))]), coordsA=plot1.transData,
                            xyB=(hour_list2.index('09:00')/2, num_requests2[int(hour_list2.index('09:00'))]), coordsB=plot2.transData, color='gray', linewidth=0.5, linestyle='--')
    fig.add_artist(conn2)
    conn3 = ConnectionPatch(xyA=(hour_list.index('11:00')/2, num_requests[int(hour_list.index('11:00'))]), coordsA=plot1.transData,
                            xyB=(hour_list3.index('11:00')/2, num_requests3[int(hour_list3.index('11:00'))]), coordsB=plot3.transData, color='gray', linewidth=0.5, linestyle='--')
    fig.add_artist(conn3)
    conn4 = ConnectionPatch(xyA=(hour_list.index('15:00')/2, num_requests[int(hour_list.index('15:00'))]), coordsA=plot1.transData,
                            xyB=(hour_list3.index('15:00')/2, num_requests3[int(hour_list3.index('15:00'))]), coordsB=plot3.transData, color='gray', linewidth=0.5, linestyle='--')
    fig.add_artist(conn4)


    # Adjust layout
    plt.tight_layout()

    image_dir = parent_folder + "Zoom_Images/"
    file_name = f"{limit}_{year}{month}{day}_requests.png"
    os.makedirs(image_dir, exist_ok=True)
    plt.savefig(image_dir + file_name, dpi=300, bbox_inches="tight")

    plt.close(fig)
    return average_requests

def plot_customer_arrival(df_dataset, parent_folder, time_origin):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day

    df = df_dataset.copy()  # Safe local copy

    df['count'] = 1
    df['tpep_pickup_datetime'] = pd.to_datetime(df['tpep_pickup_datetime'])

    # Group data into 30-second intervals
    df_grouped = df.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='30s')).sum(numeric_only=True)

    # Extract hours and minutes from the index
    hour_list = [d.strftime('%H:%M') for d in df_grouped.index]

    fig, ax = plt.subplots(figsize=c.plot_config["fig_size_half"])
    num_customers = df_grouped['passenger_count'].tolist()

    ax.plot(hour_list, num_customers, color='orange')

    vertical_lines = ['07:00', '23:00']

    # Shade the areas between 07:00-09:00 and 12:00-14:00
    ax.axvspan('07:00', '23:00', color='lightgray', alpha=0.3)

    for time in vertical_lines:
        ax.axvline(x=time, color='gray', linewidth=0.5, linestyle='--')

    # Calculate the average number of customers and plot the horizontal line
    average_customers = np.mean(num_customers)
    plt.axhline(y=average_customers, color='maroon', label=f'Average: {average_customers:.1f}', linewidth=1)

    limit = 24
    hour_list.append('24:00')
    hour_list.append('24:00')
    num_ticks = len(hour_list) // limit  # Assuming 24 hours
    x_ticks = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    x_labels = [hour_list[i] for i in range(0, len(hour_list), num_ticks)]
    # Add the last time point to the x-ticks and labels
    x_ticks.append(hour_list[-1])
    x_labels.append(hour_list[-1])
    plt.xticks(rotation=40)
    plt.tick_params(axis='both', labelsize=c.plot_config["tick_label_fsize"])
    plt.xticks(x_ticks, x_labels)

#    plt.xlabel('Time', fontsize=text_size, fontweight='bold')
    ax.set_ylabel('Number of customers', fontsize=c.plot_config["axis_label_fsize"], fontweight='bold')
    ax.grid(axis='x', linestyle='--', color='lightgray')
    ax.legend(loc='lower right', fontsize=c.plot_config["legend_fsize"], edgecolor=c.plot_config["legend_edgecolor"],
              framealpha=1.0, facecolor='white')

    # Set x-axis limits to exclude the empty space before the first x-tick
    ax.set_xlim(x_ticks[0], x_ticks[-1])
#    ax.set_ylim(0, 405)

    # Adjust layout
    plt.tight_layout()

    image_dir = parent_folder + "Customer_Arrival/"
    file_name = f"{limit}_{year}{month}{day}_customers.png"
    os.makedirs(image_dir, exist_ok=True)
    plt.savefig(image_dir + file_name, dpi=300, bbox_inches="tight")

    plt.close(fig)
    return average_customers


def plot_map_request_cells(district_network, dataset, parent_folder, file_name=None):
    """ PLOT POCKUP POINTS """
    fig, ax = plot_districts(district_network, mapsize=c.plot_config["district_map_size"])
    points = dataset["pickup_ID"]
    request_cells = []
    for item in points:
        request_cells.append([item, district_network.cell_to_longitude[item], district_network.cell_to_latitude[item]])
    request_cells = pd.DataFrame(request_cells, columns=['cell_ID', 'longitude', 'latitude'])
    request_points = request_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(request_points['latitude'])
    x = np.array(request_points['longitude'])
    cell_size = np.array(request_points['cell_size'])
    plt.scatter(x, y, c='green', s=cell_size/10, alpha=0.35)

    image_dir = parent_folder + "Request_Cells/"
    os.makedirs(image_dir, exist_ok=True)
    fig.savefig(image_dir + file_name + '_pickup', dpi=300, bbox_inches="tight")
    plt.close(fig)

    """ PLOT DROP OFF POINT """
    fig, ax = plot_districts(district_network, mapsize=c.plot_config["district_map_size"])
    points = dataset["dropoff_ID"]
    request_cells = []
    for item in points:
        request_cells.append([item, district_network.cell_to_longitude[item], district_network.cell_to_latitude[item]])
    request_cells = pd.DataFrame(request_cells, columns=['cell_ID', 'longitude', 'latitude'])
    request_points = request_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(request_points['latitude'])
    x = np.array(request_points['longitude'])
    cell_size = np.array(request_points['cell_size'])
    plt.scatter(x, y, c='blue', s=cell_size/10, alpha=0.35)

    image_dir = parent_folder + "Request_Cells/"
    os.makedirs(image_dir, exist_ok=True)
    fig.savefig(image_dir + file_name + '_dropoff', dpi=300, bbox_inches="tight")
    plt.close(fig)

def plot_districts_fill(district_network, trip_per_district, parent_folder, file_name):
    # Compute color tones and colormap bins
    color_ton, bins, colors = calc_color(trip_per_district)
    cmap = mpl.colors.ListedColormap(colors)

    # Ensure integer bounds and set the lower bound to zero
    bounds = [int(b) for b in bins]
    bounds[0] = 0
    norm = mpl.colors.BoundaryNorm(bounds, ncolors=cmap.N)

    # Plot districts with colored fills
    fig, ax = plot_districts(district_network, mapsize=c.plot_config["district_map_size"])
    for idx, region in enumerate(district_network.districts):
        latitudes = region.coordinates[:, 0]
        longitudes = region.coordinates[:, 1]
        ax.fill(longitudes, latitudes, color=color_ton[idx])

    # Add colorbar
    sm = mpl.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    fig.colorbar(sm, ax=ax, ticks=bounds, orientation='vertical')

    # Save figure
    output_dir = os.path.join(parent_folder, "Request_Color_Maps")
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(os.path.join(output_dir, f"{file_name}_colour.png"), dpi=300, bbox_inches="tight")
    plt.close(fig)

def calc_color(data, color=None):
    color_palettes = {
        1: ['#FFFFFF', '#f0eaf4', '#dbdaeb', '#c0c9e2', '#9cb9d9', '#73a9cf', '#4295c3', '#187cb6', '#0567a2', '#045382'],
        0: ['#FFFFFF', '#f1faba', '#d6efb3', '#abdeb7', '#73c8bd', '#40b5c4', '#2498c1', '#2072b1', '#234da0', '#1f2f87']
    }
    color_sq = color_palettes.get(color, color_palettes[0])

    np_data = np.asarray(data)
    positive_data = np_data[np_data > 0]

    # Compute quantile-based bin edges
    _, raw_bins = pd.qcut(positive_data, 10, retbins=True, duplicates='drop')
    splits = list(map(int, raw_bins))

    # Adjust bin edges for readability and monotonicity
    def adjust_bin(val, prev, unit):
        val = round(val / unit) * unit
        if val <= prev:
            val = prev + unit
        return val

    for i in range(1, len(splits)):
        if splits[i] > 10:
            if splits[i] < 100:
                splits[i] = adjust_bin(splits[i], splits[i - 1], 10)
            elif splits[i] < 1000:
                splits[i] = adjust_bin(splits[i], splits[i - 1], 100)
            elif splits[i] < 10000:
                splits[i] = adjust_bin(splits[i], splits[i - 1], 500)
            else:
                splits[i] = adjust_bin(splits[i], splits[i - 1], 1000)

    splits[0] = -1  # include zeros and negatives
    if splits[-1] > 10000:
        splits[-1] = math.ceil(splits[-1] / 5000) * 5000
    elif splits[-1] > 1000:
        splits[-1] = math.ceil(splits[-1] / 1000) * 1000
        if splits[-1] == splits[-2]:
            splits[-1] += 1000

    # Assign color index
    new_data, bins = pd.cut(data, bins=splits, labels=list(range(10)), retbins=True)

    # Map color tones
    color_ton = [color_sq[int(val)] if not pd.isna(val) else color_sq[0] for val in new_data]

    return color_ton, bins, color_sq