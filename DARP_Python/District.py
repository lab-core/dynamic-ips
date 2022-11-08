import numpy as np
import matplotlib.pyplot as plt


class District(object):
    def __init__(self, cartodb_id, name, coordinates):
        self.cartodb_id = cartodb_id
        self.name = name
        self.coordinates = coordinates
        self.cells = []

    def __str__(self):
        class_string = str(self.__class__) + ": {"
        for attribute, value in self.__dict__.items():
            class_string += str(attribute) + ": " + str(value) + ",\n"
        class_string += "}"
        return class_string

    # function that determine whether a point is inside a district polygon or not
    def point_inside_district(self, lat, long):
        n = len(self.coordinates)
        poly_matrix = np.array(self.coordinates)
        inside = False
        p1_lat, p1_long = poly_matrix[0]
        for i in range(n + 1):
            p2_lat, p2_long = poly_matrix[i % n]
            if long > min(p1_long, p2_long):
                if long <= max(p1_long, p2_long):
                    if lat <= max(p1_lat, p2_lat):
                        if p1_long != p2_long:
                            xinters = (long - p1_long) * (p2_lat - p1_lat) / (p2_long - p1_long) + p1_lat
                        if p1_lat == p2_lat or lat <= xinters:
                            inside = not inside
            p1_lat, p1_long = p2_lat, p2_long
        return inside

    def find_cells_inside(self, manhattan_cells):
        for items in manhattan_cells:
            if self.point_inside_district(items[1], items[2]):
                self.cells.append(items)
        self.cells = np.array(self.cells)

    def plot_shape(self, x_lim=None, y_lim=None, figsize=(10, 7)):
        plt.figure(figsize=figsize)
        # plotting the graphical axes where map plotting
        ax = plt.axes()
        ax.set_aspect('equal')
        x_lat = self.coordinates[:, 0]
        y_lon = self.coordinates[:, 1]
        # plotting using the derived coordinated stored in array created by numpy
        plt.plot(x_lat, y_lon)
        plt.scatter(self.cells[:, 1], self.cells[:, 2], c="orange", s=15)
        if (x_lim is not None) & (y_lim is not None):
            plt.xlim(x_lim)
            plt.ylim(y_lim)
        plt.xlabel('Latitude', fontsize=10, fontweight='bold')
        plt.ylabel('Longitude', fontsize=10, fontweight='bold')
        plt.xticks(np.arange(min(x_lat - 0.005), max(x_lat) + 0.005, 0.005))
        plt.yticks(np.arange(min(y_lon - 0.005), max(y_lon) + 0.005, 0.005))
        plt.tick_params(axis='both', labelsize=7)
        plt.tight_layout()
        plt.show(block=True)
        # use bbox (bounding box) to set plot limits
