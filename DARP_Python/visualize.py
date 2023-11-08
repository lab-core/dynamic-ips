import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns

import constants
from Network import *
import pandas as pd
import math
import datetime as dtime
import os


def calc_color(data, color=None):
    if color == 1:
        color_sq = ['#FFFFFF', '#f0eaf4', '#dbdaeb', '#c0c9e2', '#9cb9d9', '#73a9cf', '#4295c3', '#187cb6', '#0567a2',
                    '#045382']
    else:
        color_sq = ['#FFFFFF', '#f1faba', '#d6efb3', '#abdeb7', '#73c8bd', '#40b5c4', '#2498c1', '#2072b1', '#234da0',
                    '#1f2f87']
    np_data = np.array(data)
    non_zero_cuts, splits = pd.qcut(np_data[np_data > 0], 10, retbins=True, labels=False, duplicates='drop')
    splits = [int(i) for i in splits]
    for i in range(1, 10):
        if splits[i] > 10:
            if splits[i] < 100:
                splits[i] = (round(splits[i] / 10)) * 10
                if splits[i] == splits[i - 1]:
                    splits[i] += 10
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1] - splits[i] + 10
            elif splits[i] < 1000:
                splits[i] = (round(splits[i] / 100)) * 100
                if splits[i] == splits[i - 1]:
                    splits[i] += 100
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1] - splits[i] + 100
            elif splits[i] < 10000:
                splits[i] = (round(splits[i] / 500)) * 500
                if splits[i] == splits[i - 1]:
                    splits[i] += 500
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1] - splits[i] + 500
            else:
                splits[i] = (round(splits[i] / 1000)) * 1000
                if splits[i] == splits[i - 1]:
                    splits[i] += 1000
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1] - splits[i] + 1000
    splits[0] = -1
    if splits[10] > 1000:
        splits[10] = (math.ceil(splits[10] / 1000)) * 1000
        if splits[10] == splits[9]:
            splits[10] += 1000

    elif splits[10] > 10000:
        splits[10] = (math.ceil(splits[10] / 5000)) * 5000
    new_data, bins = pd.cut(data, splits, retbins=True, labels=list(range(10)))

    color_ton = []
    for val in new_data:
        color_ton.append(color_sq[val])
    return color_ton, bins, color_sq


def plot_districts(district_network, print_id=False, x_lim=None, y_lim=None, figsize=c.OS_MAP_SIZE, add_legend=False,
                   file_name=None):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect('equal')
    text_str = ""
    for region in district_network.districts:
        latitude = region.coordinates[:, 0]
        longitude = region.coordinates[:, 1]
        plt.plot(longitude, latitude, 'k', linewidth=0.7)
        if print_id is not False:
            text_y = (max(latitude) + min(latitude)) / 2
            text_x = (max(longitude) + min(longitude)) / 2
            plt.text(text_x, text_y, region.cartodb_id, fontsize=5, fontweight='bold', color='k')
        text_str = text_str + str(region.cartodb_id) + " - " + region.name + '\n'

    if (x_lim is not None) & (y_lim is not None):
        plt.xlim(x_lim)
        plt.ylim(y_lim)
    if add_legend:
        ax.text(0.03, 0.98, text_str, transform=ax.transAxes, fontsize=6, fontweight='bold', verticalalignment='top')
    plt.xlabel('Longitude', fontsize=10, fontweight='bold')
    plt.ylabel('Latitude', fontsize=10, fontweight='bold')
    plt.tick_params(axis='both', labelsize=7)
    plt.tight_layout()
    plt.xticks(
        np.arange(min(district_network.outbound_cells[:, 2] - 0.02),
                  max(district_network.outbound_cells[:, 2]) + 0.02, 0.02))
    plt.yticks(
        np.arange(min(district_network.outbound_cells[:, 1] - 0.02),
                  max(district_network.outbound_cells[:, 1]) + 0.02, 0.02))
    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Map_Cells"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name)
    return fig, ax


def plot_map_cells(district_network, print_id, pause=False, file_name=None):
    """SELECT INBOUND DISTRICTS"""
    inbound_districts = []
    for ID in c.INBOUND_DISTRICTS:
        for item in district_network.districts:
            if item.cartodb_id == ID:
                inbound_districts.append(item)
                break

    fig, ax = plot_districts(district_network, print_id)
    text_str = ""
    for region in inbound_districts:
        if len(region.cells):
            plt.scatter(region.cells[:, 2], region.cells[:, 1], c="blue", s=1)
            text_str = text_str + str(region.cartodb_id) + " - " + region.name + '\n'
    #   plt.scatter(district_network.outbound_cells[:, 1], district_network.outbound_cells[:, 2], c="red", s=1)
    ax.text(1.02, 0.53, text_str, transform=ax.transAxes, fontsize=6, fontweight='bold', verticalalignment='top')
    ax.set_title("Potential Stop Locations", fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()
    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Map_Cells"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name)
    if pause:
        plt.pause(10)
    plt.close(fig)


def plot_map_request_cells(district_network, dataset, print_id, file_name=None):
    """ PLOT POCKUP POINTS """
    fig, ax = plot_districts(district_network, print_id)
    points = dataset.dataset["pickup_ID"]
    request_cells = []
    for item in points:
        request_cells.append([item, district_network.cell_to_longitude[item], district_network.cell_to_latitude[item]])
    request_cells = pd.DataFrame(request_cells, columns=['cell_ID', 'longitude', 'latitude'])
    request_points = request_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(request_points['latitude'])
    x = np.array(request_points['longitude'])
    cell_size = np.array(request_points['cell_size'])
    colors = np.random.randint(100, size=(len(x)))
    # plt.scatter(x, y, c=colors, s=cell_size, alpha=0.5, cmap='nipy_spectral')
    plt.scatter(x, y, c='green', s=cell_size, alpha=0.4)
    ax.set_title(file_name + ' pickup points', fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()

    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Request_Cells"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name + '_pickup')
    plt.close(fig)

    """ PLOT DROP OFF POINT """
    fig, ax = plot_districts(district_network, print_id)
    points = dataset.dataset["dropoff_ID"]
    request_cells = []
    for item in points:
        request_cells.append([item, district_network.cell_to_longitude[item], district_network.cell_to_latitude[item]])
    request_cells = pd.DataFrame(request_cells, columns=['cell_ID', 'longitude', 'latitude'])
    request_points = request_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(request_points['latitude'])
    x = np.array(request_points['longitude'])
    cell_size = np.array(request_points['cell_size'])
    colors = np.random.randint(100, size=(len(x)))
    # plt.scatter(x, y, c=colors, s=cell_size, alpha=0.5, cmap='nipy_spectral')
    plt.scatter(x, y, c='blue', s=cell_size, alpha=0.4)
    ax.set_title(file_name + ' dropoff points', fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()

    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Request_Cells"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name + '_dropoff')
    plt.close(fig)


def plot_map_vehicle_cells(district_network, df_vehicle, print_id, file_name=None):
    """ PLOT POCKUP POINTS """
    fig, ax = plot_districts(district_network, print_id)
    points = df_vehicle["depart_ID"].astype('int')
    vehicle_cells = []
    for item in points:
        vehicle_cells.append([item, district_network.cell_to_longitude[item], district_network.cell_to_latitude[item]])
    vehicle_cells = pd.DataFrame(vehicle_cells, columns=['cell_ID', 'longitude', 'latitude'])
    vehicle_points = vehicle_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(vehicle_points['latitude'])
    x = np.array(vehicle_points['longitude'])
    cell_size = 10 * np.array(vehicle_points['cell_size'])
    colors = np.random.randint(100, size=(len(x)))
    # plt.scatter(x, y, c=colors, s=cell_size, alpha=0.5, cmap='nipy_spectral')
    plt.scatter(x, y, c='red', s=cell_size, alpha=1)
    ax.set_title(file_name, fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()

    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Vehicle_Cells"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name)
    plt.close(fig)


def plot_unused_vehicle(district_network, instance_folder, nb_vehicles):
    root_folder = c.DATASETS_DIR + instance_folder
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith(".csv") and file.startswith("Routes_"):
                file_path = os.path.join(root, file)
                df_routes = pd.read_csv(file_path, sep=',', header=0)
                parent_folder = os.path.basename(root)
                fig, ax = plot_districts(district_network)

                routes_data = df_routes.drop(columns=['NodeID']).to_numpy(dtype='double')
                route_results = []
                for v in range(nb_vehicles):
                    route_results.append(np.array(routes_data[routes_data[:, 0] == v]))

                for v in range(nb_vehicles):
                    if len(routes_data[routes_data[:, 0] == v]) == 1:
                        route_nodes = np.array(
                            [district_network.locations[index] for index in (route_results[v][:, 4]).astype(int)])
                        plt.plot(route_nodes[:, 2], route_nodes[:, 1], marker="s", c='red', markersize=2)

                ax.set_title("Idle vehicles", fontsize=13, fontweight='bold', y=1.0,
                             pad=-14)
                fig.show()

                fig.savefig(root + "/IdleVehicles.png")
                plt.close(fig)


def plot_sink_vehicle(district_network, instance_folder, nb_vehicles):
    root_folder = c.DATASETS_DIR + instance_folder
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith(".csv") and file.startswith("Routes_"):
                file_path = os.path.join(root, file)
                df_routes = pd.read_csv(file_path, sep=',', header=0)
                parent_folder = os.path.basename(root)

                fig, ax = plot_districts(district_network)
                routes_data = df_routes.drop(columns=['NodeID']).to_numpy(dtype='double')
                route_results = []
                for v in range(nb_vehicles):
                    route_results.append(np.array(routes_data[routes_data[:, 0] == v]))

                nodes = []
                for v in range(nb_vehicles):
                    route_nodes = np.array(
                        [district_network.locations[index] for index in (route_results[v][:, 4]).astype(int)])
                    nodes.append(route_nodes[-1])
                node_mat = np.vstack(nodes)
                vehicle_cells = pd.DataFrame(node_mat, columns=['cell_ID', 'latitude', 'longitude'])
                vehicle_points = vehicle_cells.groupby(['cell_ID', 'longitude', 'latitude'])[
                    'cell_ID'].size().reset_index(
                    name='cell_size')
                y = np.array(vehicle_points['latitude'])
                x = np.array(vehicle_points['longitude'])
                cell_size = 10 * np.array(vehicle_points['cell_size'])
                plt.scatter(x, y, c='blue', s=cell_size, alpha=1)
                ax.set_title("Final position of vehicles", fontsize=13, fontweight='bold', y=1.0,
                             pad=-14)
                fig.show()
                fig.savefig(root + "/FinalVehiclesLocation.png")
                plt.close(fig)


def plot_initial_vehicle(district_network, instance_folder, nb_vehicles):
    root_folder = c.DATASETS_DIR + instance_folder
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith(".csv") and file.startswith("Routes_"):
                file_path = os.path.join(root, file)
                df_routes = pd.read_csv(file_path, sep=',', header=0)
                parent_folder = os.path.basename(root)

                fig, ax = plot_districts(district_network)
                routes_data = df_routes.drop(columns=['NodeID']).to_numpy(dtype='double')
                route_results = []
                for v in range(nb_vehicles):
                    route_results.append(np.array(routes_data[routes_data[:, 0] == v]))

                nodes = []
                for v in range(nb_vehicles):
                    route_nodes = np.array(
                        [district_network.locations[index] for index in (route_results[v][:, 4]).astype(int)])
                    nodes.append(route_nodes[0])
                node_mat = np.vstack(nodes)
                vehicle_cells = pd.DataFrame(node_mat, columns=['cell_ID', 'latitude', 'longitude'])
                vehicle_points = vehicle_cells.groupby(['cell_ID', 'longitude', 'latitude'])[
                    'cell_ID'].size().reset_index(
                    name='cell_size')
                y = np.array(vehicle_points['latitude'])
                x = np.array(vehicle_points['longitude'])
                cell_size = 10 * np.array(vehicle_points['cell_size'])
                plt.scatter(x, y, c='green', s=cell_size, alpha=1)
                ax.set_title("Initial position of vehicles", fontsize=13, fontweight='bold', y=1.0,
                             pad=-14)
                fig.show()
                fig.savefig(root + "/InitialVehiclesLocation.png")
                plt.close(fig)


# if len(routes_data[routes_data[:, 0] == v]) == 1:
def plot_districts_fill_by_id(district_network, print_id, color_ton, colors, bins, pause=False, file_name=None):
    cmap = mpl.colors.ListedColormap(colors)
    bounds = [int(bins) for bins in bins]
    bounds[0] = 0
    fig, ax = plot_districts(district_network, print_id)
    text_str = ""
    for index, item in enumerate(district_network.districts):
        latitude = item.coordinates[:, 0]
        longitude = item.coordinates[:, 1]
        ax.fill(longitude, latitude, color_ton[index])
        text_str = text_str + str(item.cartodb_id) + " - " + item.name + '\n'

    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    my_map = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap, norm=norm), ticks=bounds, orientation='vertical')
    my_map.ax.set_yticklabels(bounds, fontsize=8, fontweight='bold')
    ax.text(0.03, 0.98, text_str, transform=ax.transAxes, fontsize=6, fontweight='bold', verticalalignment='top')
    ax.set_title(file_name, fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.canvas.draw()
    if file_name is not None:
        parent_folder = c.DAYS_DIR + "Maps"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        fig.savefig(image_dir + file_name)
    if pause:
        plt.pause(10)
    plt.close(fig)


def plot_districts_by_nb_trips(trip_per_district, district_network, print_id, pause=False, file_name=None):
    color_ton, bins, colors = calc_color(trip_per_district)
    plot_districts_fill_by_id(district_network, print_id, color_ton, colors, bins, pause=pause, file_name=file_name)


def show_request_per_hr(df_dataset, time_origin, save_image=False, os_type=None, pause=False):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    if os_type is None:
        os_type = "osx"
    title_size = c.WIN_TITLE_SIZE
    text_size = c.WIN_TEXT_SIZE
    if os_type == "osx":
        title_size = c.OS_TITLE_SIZE
        text_size = c.OS_TEXT_SIZE
    df_dataset['count'] = 1
    df_dataset['tpep_pickup_datetime'] = pd.to_datetime(df_dataset['tpep_pickup_datetime'])
    df_grouped = df_dataset.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='H')).sum(numeric_only=True)

    hour_list = tuple([dtime.datetime.strftime(d, '%H:%M') for d in df_grouped.index.tolist()])
    y_pos = np.arange(len(hour_list))
    num_requests = df_grouped['count'].tolist()
    if os_type == "windows":
        fig, ax = plt.subplots(figsize=c.WIN_FIG_SIZE)
    else:
        fig, ax = plt.subplots(figsize=c.OS_FIG_SIZE)
    title = "Number of arrival requests per hour (" + str(year) + "-" + str(month) + "-" + str(day) + ")"
    ax.set_title(title, fontsize=title_size, fontweight='bold')
    ax.bar(y_pos, num_requests, align='center', alpha=1, color='yellowgreen', width=0.6)
    for i in range(len(hour_list)):
        ax.annotate(num_requests[i], (0 + i, num_requests[i] + num_requests[4] * 0.2), weight='bold',
                    size=text_size, ha='center', va='center', color='darkgreen')

    ax.tick_params(axis='both', labelsize=text_size)

    ax.set_xticks(y_pos)
    ax.set_xticklabels(hour_list)

    ax.set_ylabel('Number of requests', fontsize=text_size, fontweight='bold')

    if save_image:
        parent_folder = c.DAYS_DIR + "Images_trip"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        file_name = str(year) + str(month) + str(day) + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)
    plt.close(fig)


def show_customer_per_hr(df_dataset, time_origin, save_image=False, os_type=None, pause=False):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    if os_type is None:
        os_type = "osx"
    title_size = c.WIN_TITLE_SIZE
    text_size = c.WIN_TEXT_SIZE
    if os_type == "osx":
        title_size = c.OS_TITLE_SIZE
        text_size = c.OS_TEXT_SIZE
    df_dataset['count'] = df_dataset['passenger_count']
    df_dataset['tpep_pickup_datetime'] = pd.to_datetime(df_dataset['tpep_pickup_datetime'])
    df_grouped = df_dataset.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='H')).sum(numeric_only=True)

    hour_list = tuple([dtime.datetime.strftime(d, '%H:%M') for d in df_grouped.index.tolist()])
    y_pos = np.arange(len(hour_list))
    num_requests = df_grouped['count'].tolist()
    if os_type == "windows":
        fig, ax = plt.subplots(figsize=c.WIN_FIG_SIZE)
    else:
        fig, ax = plt.subplots(figsize=c.OS_FIG_SIZE)
    title = "Number of arrival requests per hour (" + str(year) + "-" + str(month) + "-" + str(day) + ")"
    ax.set_title(title, fontsize=title_size, fontweight='bold')
    ax.bar(y_pos, num_requests, align='center', alpha=1, color='orange', width=0.6)
    for i in range(len(hour_list)):
        ax.annotate(num_requests[i], (0 + i, num_requests[i] + num_requests[4] * 0.2), weight='bold',
                    size=text_size, ha='center', va='center', color='maroon')

    ax.tick_params(axis='both', labelsize=text_size)

    ax.set_xticks(y_pos)
    ax.set_xticklabels(hour_list)

    ax.set_ylabel('Number of requests', fontsize=text_size, fontweight='bold')

    if save_image:
        parent_folder = c.DAYS_DIR + "Images_customers"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        file_name = str(year) + str(month) + str(day) + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)
    plt.close(fig)


def show_dataset_per_hr(df_dataset, time_origin, save_image=False, os_type=None, pause=False):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    if os_type is None:
        os_type = "osx"
    title_size = c.WIN_TITLE_SIZE
    text_size = c.WIN_TEXT_SIZE
    if os_type == "osx":
        title_size = c.OS_TITLE_SIZE
        text_size = c.OS_TEXT_SIZE
    df_dataset['count'] = 1
    df_dataset['tpep_pickup_datetime'] = pd.to_datetime(df_dataset['tpep_pickup_datetime'])
    df_grouped = df_dataset.groupby(pd.Grouper(key='tpep_pickup_datetime', freq='H')).sum(numeric_only=True)

    hour_list = tuple([dtime.datetime.strftime(d, '%H:%M') for d in df_grouped.index.tolist()])
    y_pos = np.arange(len(hour_list))
    num_requests = df_grouped['count'].tolist()
    num_customers = df_grouped['passenger_count'].tolist()
    if os_type == "windows":
        fig = plt.figure(figsize=c.WIN_FIG_SIZE)
    else:
        fig = plt.figure(figsize=c.OS_FIG_SIZE)
    gs = fig.add_gridspec(2, hspace=0)
    ax = gs.subplots(sharex=True)
    title = "Number of requests/customers per hour (" + str(year) + "-" + str(month) + "-" + str(day) + ")"
    fig.suptitle(title, fontsize=title_size, fontweight='bold')
    ax[0].bar(y_pos, num_requests, align='center', alpha=1, color='yellowgreen', width=0.6)
    for i in range(len(hour_list)):
        ax[0].annotate(num_requests[i], (0 + i, num_requests[i] + num_requests[4] * 0.3), weight='bold',
                       size=text_size, ha='center', va='center', color='darkgreen')

    ax[1].bar(y_pos, num_customers, align='center', alpha=1, color='yellow', width=0.6)
    for i in range(len(hour_list)):
        ax[1].annotate(num_customers[i], (0 + i, num_customers[i] + num_customers[4] * 0.3), weight='bold',
                       size=text_size, ha='center', va='center', color='darkgreen')
    ax[0].tick_params(axis='both', labelsize=text_size)
    ax[0].set_xticks(y_pos)
    ax[0].set_xticklabels(hour_list)
    ax[0].set_ylabel('Number of requests', fontsize=text_size, fontweight='bold')

    ax[1].tick_params(axis='both', labelsize=text_size)
    ax[1].set_xticks(y_pos)
    ax[1].set_xticklabels(hour_list)
    ax[1].set_ylabel('Number of customers', fontsize=text_size, fontweight='bold')

    if save_image:
        parent_folder = c.DAYS_DIR + "Images"
        if not os.path.exists(parent_folder):
            os.mkdir(parent_folder)
        image_dir = parent_folder + "/"
        file_name = str(year) + str(month) + str(day) + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)

    plt.close(fig)


def Create_box_plot(instance_folder, algorithms, start_time, save_image=False, os_type=None, pause=False,
                    rotate=False, per_min=False, show_outliers=False):
    root_folder = c.DATASETS_DIR + instance_folder
    lighter_palette = sns.color_palette("Set3", n_colors=12, desat=0.6)
    sns.set_theme(style="ticks", palette=lighter_palette)
    results = []
    for algorithm in algorithms:
        wait_time_col = []
        for root, dirs, files in os.walk(root_folder):
            for file in files:
                if file.endswith(".csv") and file.startswith("Requests_" + algorithm):
                    print(file)
                    file_path = os.path.join(root, file)
                    data = pd.read_csv(file_path, index_col=False)
                    filtered_data = data[data['RequestTime'] >= start_time]
                    filtered_data = filtered_data.reset_index(drop=True)
                    if per_min:
                        wait_time_col.append(filtered_data[' WaitTime'] / 60)
                    else:
                        wait_time_col.append(filtered_data[' WaitTime'])

        wait_time_col.sort(key=len)

        for series in wait_time_col:
            instance = 'R' + str(len(series))
            df = pd.DataFrame(
                {'Instances': [instance] * len(series), 'Algorithm': [c.ALGORITHMS[algorithm]] * len(series),
                 'Wait_time': series})
            results.append(df)
    df_results = pd.concat(results, ignore_index=True)

    if os_type == "windows":
        fig = plt.figure(figsize=c.WIN_FIG_SIZE)
    else:
        fig = plt.figure(figsize=c.OS_FIG_SIZE)
    if show_outliers:
        sns.boxplot(x="Instances", y="Wait_time", hue="Algorithm", data=df_results, width=0.8, linewidth=0.8, dodge=0.2,
                    flierprops=dict(markerfacecolor='white', markeredgewidth=0.8, marker='o', markersize=2))
    else:
        sns.boxplot(x="Instances", y="Wait_time", hue="Algorithm", data=df_results, width=0.8, linewidth=0.8,
                    dodge=True,
                    showfliers=False)
    #   sns.despine(offset=10, trim=True)
    plt.xlabel('Instances', fontsize=10, fontweight='bold')
    if per_min:
        plt.ylabel('wait time (Minutes)', fontsize=10, fontweight='bold')
    else:
        plt.ylabel('wait time (Seconds)', fontsize=10, fontweight='bold')
    plt.legend(title="Algorithm")
    if rotate:
        plt.xticks(rotation=90)
    if save_image:
        image_dir = root_folder + "/"
        if show_outliers:
            file_name = 'boxPlot_outlier' + '.png'
        else:
            file_name = 'boxPlot' + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)

    plt.close(fig)


def Create_box_plot_by_size(instance_folder, algorithms, start_time, size, save_image=False, os_type=None, pause=False,
                            rotate=False, per_min=False, show_outliers=False):
    root_folder = c.DATASETS_DIR + instance_folder
    lighter_palette = sns.color_palette("Set3", n_colors=12, desat=0.6)
    sns.set_theme(style="ticks", palette=lighter_palette)
    results = []
    for algorithm in algorithms:
        wait_time_col = []
        for root, dirs, files in os.walk(root_folder):
            for file in files:
                if file.endswith(".csv") and file.startswith("Requests_" + algorithm):
                    file_path = os.path.join(root, file)
                    data = pd.read_csv(file_path, index_col=False)
                    filtered_data = data[data['RequestTime'] >= start_time]
                    filtered_data = filtered_data.reset_index(drop=True)
                    if c.TEST_SIZE[size][0] <= filtered_data['nbPassengers'].sum() <= c.TEST_SIZE[size][1]:
                        if per_min:
                            wait_time_col.append(filtered_data[' WaitTime'] / 60)
                        else:
                            wait_time_col.append(filtered_data[' WaitTime'])

        wait_time_col.sort(key=len)

        for series in wait_time_col:
            instance = 'R' + str(len(series))
            df = pd.DataFrame(
                {'Instances': [instance] * len(series), 'Algorithm': [c.ALGORITHMS[algorithm]] * len(series),
                 'Wait_time': series})
            results.append(df)
    df_results = pd.concat(results, ignore_index=True)

    if os_type == "windows":
        fig = plt.figure(figsize=c.WIN_FIG_SIZE)
    else:
        fig = plt.figure(figsize=c.OS_FIG_SIZE)
    if show_outliers:
        sns.boxplot(x="Instances", y="Wait_time", hue="Algorithm", data=df_results, width=0.8, linewidth=0.8, dodge=0.2,
                    flierprops=dict(markerfacecolor='white', markeredgewidth=0.8, marker='o', markersize=2))
    else:
        sns.boxplot(x="Instances", y="Wait_time", hue="Algorithm", data=df_results, width=0.8, linewidth=0.8,
                    dodge=True,
                    showfliers=False)
    #   sns.despine(offset=10, trim=True)
    plt.xlabel('Instances', fontsize=10, fontweight='bold')
    if per_min:
        plt.ylabel('wait time (Minutes)', fontsize=10, fontweight='bold')
    else:
        plt.ylabel('wait time (Seconds)', fontsize=10, fontweight='bold')
    plt.legend(title="Algorithm")

    plt.title("Instances with the number of customers " + c.TEST_GROUP[size], fontsize=c.OS_TITLE_SIZE,
              fontweight='bold')
    if rotate:
        plt.xticks(rotation=90)
    if save_image:
        image_dir = root_folder + "/"
        if show_outliers:
            file_name = 'boxPlot_outlier_' + size + '.png'
        else:
            file_name = 'boxPlot_' + size + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)

    plt.close(fig)


def Create_histogram(instance_folder, algorithms, start_time, save_image=False, os_type=None, pause=False,
                     log_scale=False):
    root_folder = c.DATASETS_DIR + instance_folder
    results = []
    for algorithm in algorithms:
        wait_time_col = []
        num_passenger = []
        for root, dirs, files in os.walk(root_folder):
            for file in files:
                if file.endswith(".csv") and file.startswith("Requests_" + algorithm):
                    file_path = os.path.join(root, file)
                    data = pd.read_csv(file_path, index_col=False)
                    filtered_data = data[data['RequestTime'] >= start_time]
                    filtered_data = filtered_data.reset_index(drop=True)
                    wait_time_col.append(filtered_data[' WaitTime'] / 60)
                    num_passenger.append(filtered_data['nbPassengers'])

        for i in range(len(wait_time_col)):
            instance = 'R' + str(len(wait_time_col[i]))
            df = pd.DataFrame({'Instances': [instance] * len(wait_time_col[i]),
                               'Algorithm': [c.ALGORITHMS[algorithm]] * len(wait_time_col[i]),
                               'Wait_time': wait_time_col[i],
                               'nb_passengers': num_passenger[i]})
            results.append(df)

    df_results = pd.concat(results, ignore_index=True)

    if os_type == "windows":
        fig = plt.figure(figsize=c.WIN_FIG_SIZE)
    else:
        fig = plt.figure(figsize=c.OS_FIG_SIZE)

    max_value = df_results['Wait_time'].max()
    sns.histplot(data=df_results, x="Wait_time", hue='Algorithm', binwidth=0.5, palette="deep", element="step",
                 weights='nb_passengers')
    if log_scale:
        plt.yscale('log')
    plt.xlabel('wait time (Minutes)', fontsize=10, fontweight='bold')
    plt.xticks(np.arange(0, max_value, step=2))
    plt.ylabel('number of passengers', fontsize=10, fontweight='bold')

    if save_image:
        image_dir = root_folder + "/"
        if log_scale:
            file_name = 'histogram_log' + '.png'
        else:
            file_name = 'histogram' + '.png'
        fig.savefig(image_dir + file_name)

    fig.show()
    if pause:
        plt.pause(10)

    plt.close(fig)


def Create_histogram_by_file(instance_folder, algorithms, start_time, save_image=False, os_type=None, pause=False):
    root_folder = c.DATASETS_DIR + instance_folder
    results = []
    for test_folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, test_folder)
        if os.path.isdir(folder_path):
            for algorithm in algorithms:
                wait_time_col = []
                num_passenger = []
                for root, dirs, files in os.walk(folder_path):
                    for file in files:
                        if file.endswith(".csv") and file.startswith("Requests_" + algorithm):
                            if file.endswith(".csv") and file.startswith("Requests_" + algorithm):
                                file_path = os.path.join(root, file)
                                data = pd.read_csv(file_path, index_col=False)
                                filtered_data = data[data['RequestTime'] >= start_time]
                                filtered_data = filtered_data.reset_index(drop=True)
                                wait_time_col.append(filtered_data[' WaitTime'] / 60)
                                num_passenger.append(filtered_data['nbPassengers'])

                for i in range(len(wait_time_col)):
                    instance = 'R' + str(len(wait_time_col[i]))
                    df = pd.DataFrame({'Instances': [instance] * len(wait_time_col[i]),
                                       'Algorithm': [c.ALGORITHMS[algorithm]] * len(wait_time_col[i]),
                                       'Wait_time': wait_time_col[i],
                                       'nb_passengers': num_passenger[i]})
                    results.append(df)

            df_results = pd.concat(results, ignore_index=True)

            if os_type == "windows":
                fig = plt.figure(figsize=c.WIN_FIG_SIZE)
            else:
                fig = plt.figure(figsize=c.OS_FIG_SIZE)

            max_value = df_results['Wait_time'].max()
            sns.histplot(data=df_results, x="Wait_time", hue='Algorithm', binwidth=0.5, palette="deep", element="step",
                         weights='nb_passengers', binrange=(0, 60))

            plt.xlabel('wait time (Minutes)', fontsize=10, fontweight='bold')
            plt.xticks(np.arange(0, 60, step=2))
            plt.ylabel('number of passengers', fontsize=10, fontweight='bold')

            if save_image:
                image_dir = folder_path + "/"
                file_name = 'histogram_' + algorithms[0] + '_' + instance + '.png'
                fig.savefig(image_dir + file_name)

                plt.yscale('log')
                file_name = 'histogram_log_' + algorithms[0] + '_' + instance + '.png'
                fig.savefig(image_dir + file_name)

            fig.show()
            if pause:
                plt.pause(10)

            plt.close(fig)


def plot_vehicle_states(district_network, instance_folder, nb_vehicles):
    vf.plot_sink_vehicle(district_network, instance_folder, nb_vehicles)
    vf.plot_unused_vehicle(district_network, instance_folder, nb_vehicles)
    vf.plot_initial_vehicle(district_network, instance_folder, nb_vehicles)


def Create_waitime_plot(instance_folder, save_image=False, os_type=None, pause=False, Title=False):
    root_folder = c.DATASETS_DIR + instance_folder
    for test_folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, test_folder)
        if os.path.isdir(folder_path):
            for run_folder in os.listdir(folder_path):
                run_folder_path = os.path.join(folder_path, run_folder)
                if os.path.isdir(folder_path) and run_folder.startswith("ANYTIME_") and not run_folder.startswith(
                        "ANYTIME_GREEDY"):
                    for root, dirs, files in os.walk(run_folder_path):
                        for file in files:
                            if file.endswith(".csv") and file.startswith("summary_"):
                                file_path = os.path.join(root, file)
                                data = pd.read_csv(file_path, index_col=False)
                                algorithm = "A_" + data['Algorithm'][0]
                                instance = 'R' + str(data['# (Lim) served Req.'][0])

                                file = "epochRuntime_" + algorithm + ".csv"
                                file_path = os.path.join(run_folder_path, file)
                                data = pd.read_csv(file_path, index_col=False)
                                x = data['ElapsedTime'] / 3600
                                y_wait_time = data['waitTime'] / data['nbRequests']
                                y_nb_requests = data['nbRequests']

                                if os_type == "windows":
                                    fig, ax1 = plt.subplots(figsize=c.WIN_FIG_SIZE)
                                else:
                                    fig, ax1 = plt.subplots(figsize=c.OS_FIG_SIZE)
                                sns.set_palette("deep")
                                ax1.plot(x, y_wait_time, label='avg. wait time(s)', color=sns.color_palette()[1])
                                ax1.set_xlabel('elapsed time (hour)', fontsize=10, fontweight='bold')
                                ax1.set_ylabel('time (Seconds)', color=sns.color_palette()[1], fontsize=12,
                                               fontweight='bold')
                                plt.grid(True, color='lightgray', linestyle='--')
                                ax2 = ax1.twinx()
                                ax2.plot(x, y_nb_requests, label='# Requests', color=sns.color_palette()[3])
                                ax2.set_ylabel('number of requests', color=sns.color_palette()[3], fontsize=12,
                                               fontweight='bold')

                                ax1.set_xlim(0, 4)
                                maximum = (data['waitTime'] / data['nbRequests']).max()
                                ax1.set_ylim(0, (maximum // 20 + 1) * 20)
                                ax2.set_ylim(0, (data['nbRequests'].max() // 100 + 1) * 100)

                                lines1, labels1 = ax1.get_legend_handles_labels()
                                lines2, labels2 = ax2.get_legend_handles_labels()
                                lines = lines1 + lines2
                                labels = labels1 + labels2
                                ax1.legend(lines, labels, loc='upper left')

                                if Title:
                                    plt.title("Instance " + instance, fontsize=13, fontweight='bold')

                                if save_image:
                                    image_dir = run_folder_path + "/"
                                    file_name = 'wait_time' + '_' + instance + '.png'
                                    fig.savefig(image_dir + file_name)

                                fig.show()
                                if pause:
                                    plt.pause(10)

                                plt.close(fig)


def Create_runtime_plot(instance_folder, save_image=False, os_type=None, pause=False, Title=False):
    root_folder = c.DATASETS_DIR + instance_folder
    for test_folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, test_folder)
        if os.path.isdir(folder_path):
            for run_folder in os.listdir(folder_path):
                run_folder_path = os.path.join(folder_path, run_folder)
                if os.path.isdir(folder_path) and run_folder.startswith("ANYTIME_") and not run_folder.startswith(
                        "ANYTIME_GREEDY"):
                    for root, dirs, files in os.walk(run_folder_path):
                        for file in files:
                            if file.endswith(".csv") and file.startswith("summary_"):
                                file_path = os.path.join(root, file)
                                data = pd.read_csv(file_path, index_col=False)
                                algorithm = "A_" + data['Algorithm'][0]
                                instance = 'R' + str(data['# (Lim) served Req.'][0])

                                file = "epochRuntime_" + algorithm + ".csv"
                                file_path = os.path.join(run_folder_path, file)
                                data = pd.read_csv(file_path, index_col=False)
                                x = data['ElapsedTime'] / 3600
                                y_epoch = data['EpochRuntime']
                                y_MP = data[' MP_Runtime']
                                y_nb_requests = data['nbRequests']

                                if os_type == "windows":
                                    fig, ax1 = plt.subplots(figsize=c.WIN_FIG_SIZE)
                                else:
                                    fig, ax1 = plt.subplots(figsize=c.OS_FIG_SIZE)
                                sns.set_palette("deep")
                                ax1.plot(x, y_epoch, label='epoch size(s)', color=sns.color_palette()[0])
                                ax1.plot(x, y_MP, label='Master run time(s)', color=sns.color_palette()[2])
                                ax1.set_xlabel('elapsed time (hour)', fontsize=12, fontweight='bold')
                                ax1.set_ylabel('time (Seconds)', fontsize=12, fontweight='bold')
                                plt.grid(True, color='lightgray', linestyle='--')
                                ax2 = ax1.twinx()
                                ax2.plot(x, y_nb_requests, label='# Requests', color=sns.color_palette()[3])
                                ax2.set_ylabel('number of requests', color=sns.color_palette()[3], fontsize=12,
                                               fontweight='bold')

                                ax1.set_xlim(0, 4)
                                if data['EpochRuntime'].max() > 2:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 5 + 1) * 5)
                                else:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 0.5 + 1) * 0.5)
                                ax2.set_ylim(0, (data['nbRequests'].max() // 100 + 1) * 100)

                                lines1, labels1 = ax1.get_legend_handles_labels()
                                lines2, labels2 = ax2.get_legend_handles_labels()
                                lines = lines1 + lines2
                                labels = labels1 + labels2
                                ax1.legend(lines, labels, loc='upper left')

                                if Title:
                                    plt.title("Instance " + instance, fontsize=13, fontweight='bold')

                                if save_image:
                                    image_dir = run_folder_path + "/"
                                    file_name = 'runtime' + '_' + instance + '.png'
                                    fig.savefig(image_dir + file_name)

                                fig.show()
                                if pause:
                                    plt.pause(10)

                                plt.close(fig)


def Create_time_plot(instance_folder, save_image=False, os_type=None, pause=False, Title=False):
    root_folder = c.DATASETS_DIR + instance_folder
    for test_folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, test_folder)
        if os.path.isdir(folder_path):
            for run_folder in os.listdir(folder_path):
                run_folder_path = os.path.join(folder_path, run_folder)
                if os.path.isdir(folder_path) and run_folder.startswith("ANYTIME_") and not run_folder.startswith(
                        "ANYTIME_GREEDY"):
                    for root, dirs, files in os.walk(run_folder_path):
                        for file in files:
                            if file.endswith(".csv") and file.startswith("summary_"):
                                file_path = os.path.join(root, file)
                                data = pd.read_csv(file_path, index_col=False)
                                algorithm = "A_" + data['Algorithm'][0]
                                instance = 'R' + str(data['# (Lim) served Req.'][0])

                                file = "epochRuntime_" + algorithm + ".csv"
                                file_path = os.path.join(run_folder_path, file)
                                data = pd.read_csv(file_path, index_col=False)
                                x = data['ElapsedTime'] / 3600
                                y_epoch = data['EpochRuntime']
                                y_MP = data[' MP_Runtime']
                                y_wait_time = data['waitTime'] / data['nbRequests']
                                y_nb_requests = data['nbRequests']

                                if os_type == "windows":
                                    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=c.WIN_FIG_SIZE)
                                else:
                                    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=c.OS_FIG_SIZE)

                                sns.set_palette("deep")

                                # first plot for runtimes
                                plt.grid(True, color='lightgray')
                                ax1.plot(x, y_epoch, label='epoch size(s)', color=sns.color_palette()[0])
                                ax1.plot(x, y_MP, label='Master run time(s)', color=sns.color_palette()[2])
                                #        ax1.set_xlabel('elapsed time (hour)', fontsize=10, fontweight='bold')
                                ax1.set_ylabel('time (Seconds)', fontsize=12, fontweight='bold')
                                ax1.grid(True, color='lightgray', linestyle='--')

                                ax1twin = ax1.twinx()
                                ax1twin.plot(x, y_nb_requests, label='# Requests', color=sns.color_palette()[3])
                                ax1twin.set_ylabel('number of requests', color=sns.color_palette()[3], fontsize=12,
                                                   fontweight='bold')

                                ax1.set_xlim(0, 4)
                                if data['EpochRuntime'].max() > 2:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 5 + 1) * 5)
                                else:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 0.5 + 1) * 0.5)
                                ax1twin.set_ylim(0, (data['nbRequests'].max() // 100 + 1) * 100)

                                if Title:
                                    ax1.set_title("Instance " + instance, fontsize=13, fontweight='bold')

                                lines1, labels1 = ax1.get_legend_handles_labels()
                                lines2, labels2 = ax1twin.get_legend_handles_labels()

                                lines = lines1 + lines2
                                labels = labels1 + labels2
                                ax1.legend(lines, labels, loc='upper left')

                                # second plot for objective values
                                ax2.plot(x, y_wait_time, label='avg. wait time(s)', color=sns.color_palette()[1])
                                ax2.set_xlabel('elapsed time (hour)', fontsize=12, fontweight='bold')
                                ax2.set_ylabel('time (Seconds)', fontsize=12,
                                               fontweight='bold')

                                ax2.set_xlim(0, 4)
                                maximum = (data['waitTime'] / data['nbRequests']).max()
                                ax2.set_ylim(0, (maximum // 20 + 1) * 20)

                                lines3, labels3 = ax2.get_legend_handles_labels()
                                ax2.legend(lines3, labels3, loc='upper left')

                                plt.subplots_adjust(hspace=0.05)
                                plt.tight_layout()

                                if save_image:
                                    image_dir = run_folder_path + "/"
                                    file_name = 'runtime_double' + '_' + instance + '.png'
                                    fig.savefig(image_dir + file_name)

                                fig.show()
                                if pause:
                                    plt.pause(10)

                                plt.close(fig)


def Create_time_plot1(instance_folder, save_image=False, os_type=None, pause=False, Title=False):
    root_folder = c.DATASETS_DIR + instance_folder
    for test_folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, test_folder)
        if os.path.isdir(folder_path):
            for run_folder in os.listdir(folder_path):
                run_folder_path = os.path.join(folder_path, run_folder)
                if os.path.isdir(folder_path) and run_folder.startswith("ANYTIME_") and not run_folder.startswith(
                        "ANYTIME_GREEDY"):
                    for root, dirs, files in os.walk(run_folder_path):
                        for file in files:
                            if file.endswith(".csv") and file.startswith("summary_"):
                                file_path = os.path.join(root, file)
                                data = pd.read_csv(file_path, index_col=False)
                                algorithm = "A_" + data['Algorithm'][0]
                                instance = 'R' + str(data['# (Lim) served Req.'][0])

                                file = "epochRuntime_" + algorithm + ".csv"
                                file_path = os.path.join(run_folder_path, file)
                                data = pd.read_csv(file_path, index_col=False)
                                x = data['ElapsedTime'] / 3600
                                y_epoch = data['EpochRuntime']
                                y_MP = data[' MP_Runtime']
                                y_wait_time = data['waitTime'] / data['nbRequests']
                                y_nb_requests = data['nbRequests']

                                if os_type == "windows":
                                    fig = plt.figure(figsize=c.WIN_FIG_SIZE)
                                else:
                                    fig = plt.figure(figsize=c.OS_FIG_SIZE)
                                gs = fig.add_gridspec(2, 1, hspace=0, height_ratios=[1, 1])
                                ax1 = plt.subplot(gs[0])

                                sns.set_palette("deep")

                                # first plot for runtimes
                                plt.grid(True, color='lightgray')
                                ax1.plot(x, y_epoch, label='epoch size(s)', color=sns.color_palette()[0])
                                ax1.plot(x, y_MP, label='Master run time(s)', color=sns.color_palette()[2])
                                #        ax1.set_xlabel('elapsed time (hour)', fontsize=10, fontweight='bold')
                                ax1.set_ylabel('time (Seconds)', fontsize=12, fontweight='bold')
                                ax1.grid(True, color='lightgray', linestyle='--')

                                ax1twin = ax1.twinx()
                                ax1twin.plot(x, y_nb_requests, label='# Requests', color=sns.color_palette()[1])
                                ax1twin.set_ylabel('number of requests', color=sns.color_palette()[1], fontsize=12,
                                                   fontweight='bold')

                                ax1.set_xlim(0, 4)
                                if data['EpochRuntime'].max() > 2:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 5 + 1) * 5)
                                else:
                                    ax1.set_ylim(0, (data['EpochRuntime'].max() // 0.5 + 1) * 0.5)
                                ax1twin.set_ylim(0, (data['nbRequests'].max() // 100 + 1) * 100)

                                if Title:
                                    ax1.set_title("Instance " + instance, fontsize=13, fontweight='bold')

                                lines1, labels1 = ax1.get_legend_handles_labels()
                                lines2, labels2 = ax1twin.get_legend_handles_labels()

                                lines = lines1 + lines2
                                labels = labels1 + labels2
                                ax1.legend(lines, labels, loc='upper left')

                                # second plot for objective values
                                ax2 = plt.subplot(gs[1], sharex=ax1)
                                ax2.plot(x, y_wait_time, label='avg. wait time(s)', color=sns.color_palette()[3])
                                ax2.set_xlabel('elapsed time (hour)', fontsize=10, fontweight='bold')
                                ax2.set_ylabel('time (Seconds)', fontsize=12,
                                               fontweight='bold')

                                ax2.set_xlim(0, 4)
                                maximum = (data['waitTime'] / data['nbRequests']).max()
                                ax2.set_ylim(0, (maximum // 20 + 1) * 20)

                                lines3, labels3 = ax2.get_legend_handles_labels()
                                ax2.legend(lines3, labels3, loc='upper left')

                                plt.tight_layout()

                                if save_image:
                                    image_dir = run_folder_path + "/"
                                    file_name = 'runtime_double' + '_' + instance + '.png'
                                    fig.savefig(image_dir + file_name)

                                fig.show()
                                if pause:
                                    plt.pause(10)

                                plt.close(fig)


def create_plots(instance_folder, algorithms, start_time, save_image=False, os_type=None, pause=False, Title=False):
    for algo in ["A_MP_MIP", "A_MP_CG", "A_MP_ISUD"]:
        if algo != "A_GREEDY":
            Create_histogram_by_file(instance_folder=instance_folder, start_time=start_time,
                                     algorithms=[algo, "A_GREEDY"], save_image=save_image, os_type=os_type, pause=pause)

    for sizes in ["small", "medium", "large"]:
        for status in [True, False]:
            Create_box_plot_by_size(instance_folder=instance_folder, algorithms=algorithms, start_time=start_time,
                                    save_image=save_image, size=sizes, os_type=os_type, pause=pause, rotate=False,
                                    per_min=True, show_outliers=status)

    Create_runtime_plot(instance_folder=instance_folder, save_image=save_image, os_type=os_type, pause=pause,
                        Title=Title)
    Create_waitime_plot(instance_folder=instance_folder, save_image=save_image, os_type=os_type, pause=pause,
                        Title=Title)
    Create_time_plot(instance_folder=instance_folder, save_image=save_image, os_type=os_type, pause=pause, Title=Title)
