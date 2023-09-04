import matplotlib as mpl
import matplotlib.pyplot as plt

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
                    splits[i] += splits[i - 1]-splits[i]+10
            elif splits[i] < 1000:
                splits[i] = (round(splits[i] / 100)) * 100
                if splits[i] == splits[i - 1]:
                    splits[i] += 100
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1]-splits[i]+100
            elif splits[i] < 10000:
                splits[i] = (round(splits[i] / 500)) * 500
                if splits[i] == splits[i - 1]:
                    splits[i] += 500
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1]-splits[i]+500
            else:
                splits[i] = (round(splits[i] / 1000)) * 1000
                if splits[i] == splits[i - 1]:
                    splits[i] += 1000
                if splits[i] < splits[i - 1]:
                    splits[i] += splits[i - 1]-splits[i]+1000
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


def plot_unused_vehicle(district_network, result_file, instance_name, instance_folder, nb_vehicles, method):
    result_dir = c.DATASETS_DIR + instance_folder + "/" + instance_name + "/" + result_file
    fig, ax = plot_districts(district_network)
    routes_filename = result_dir + "/Routes_" + method + ".csv"
    df_routes = pd.read_csv(routes_filename, sep=',', header=0)
    routes_data = df_routes.drop(columns=['NodeID']).to_numpy(dtype='double')
    route_results = []
    for v in range(nb_vehicles):
        route_results.append(np.array(routes_data[routes_data[:, 0] == v]))

    for v in range(nb_vehicles):
        if len(routes_data[routes_data[:, 0] == v]) == 1:
            route_nodes = np.array(
                [district_network.locations[index] for index in (route_results[v][:, 4]).astype(int)])
            plt.plot(route_nodes[:, 2], route_nodes[:, 1], marker="s", c='red', markersize=2)

    ax.set_title("Idle Vehicles in test " + instance_name, fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()

    fig.savefig(result_dir + "/IdleVehicles.png")
    plt.close(fig)


def plot_sink_vehicle(district_network, result_file, instance_name, instance_folder, nb_vehicles, method):
    result_dir = c.DATASETS_DIR + instance_folder + "/" + instance_name + "/" + result_file
    fig, ax = plot_districts(district_network)
    routes_filename = result_dir + "/Routes_" + method + ".csv"
    df_routes = pd.read_csv(routes_filename, sep=',', header=0)
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
    vehicle_points = vehicle_cells.groupby(['cell_ID', 'longitude', 'latitude'])['cell_ID'].size().reset_index(
        name='cell_size')
    y = np.array(vehicle_points['latitude'])
    x = np.array(vehicle_points['longitude'])
    cell_size = 10 * np.array(vehicle_points['cell_size'])
    plt.scatter(x, y, c='red', s=cell_size, alpha=1)
    ax.set_title("Idle Vehicles in test " + instance_name, fontsize=13, fontweight='bold', y=1.0, pad=-14)
    fig.show()

    fig.savefig(result_dir + "/IdleVehicles.png")
    plt.close(fig)

#if len(routes_data[routes_data[:, 0] == v]) == 1:
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
