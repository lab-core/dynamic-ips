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
    def point_inside_district(self, lat, lon):
        """
        Return True if (lat, lon) lies inside the polygon defined by self.coordinates.
        Uses a robust ray-casting test that avoids horizontal-edge double counts.
        Assumes coordinates are stored as (lat, lon).
        """
        # map to conventional x=lon, y=lat for the math
        x, y = float(lon), float(lat)

        poly = self.coordinates  # (N, 2) -> (lat, lon)
        if poly.ndim != 2 or poly.shape[1] != 2:
            raise ValueError("coordinates must be an array-like of (lat, lon) pairs")

        xs = poly[:, 1]  # lons
        ys = poly[:, 0]  # lats

        inside = False
        n = len(xs)

        x1, y1 = xs[0], ys[0]
        for i in range(1, n + 1):
            x2, y2 = xs[i % n], ys[i % n]

            # Edge straddles the scanline at y? (excludes purely horizontal edges)
            if (y1 > y) != (y2 > y):
                # x-coordinate where the edge crosses the scanline y
                x_intersect = x1 + (x2 - x1) * (y - y1) / (y2 - y1)

                # Toggle if intersection is to the right of the test point (even–odd rule)
                if x <= x_intersect:
                    inside = not inside

            x1, y1 = x2, y2

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
