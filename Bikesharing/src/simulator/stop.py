import copy

class Stop:
    """
    Encapsulates information about stops along vehicle routes, detailing the location,
    times of arrival and departure, and bike transfers

    Attributes
    ----------
    location: Location
        Object of type Location referring to the location of the stop
    arrival_time : float
        Time the vehicle arrives at the stop.
    depart_time : float
        Time the vehicle departs from the stop.
    nb_bike_to_pick : int
        Number of bikes intended to be picked up at the stop.
    nb_picked_bikes : int
        Number of bikes actually picked up.
    nb_bike_to_drop : int
        Number of bikes intended to be dropped off at the stop.
    nb_dropped_bikes : int
        Number of bikes actually dropped off.
    """

    def __init__(self, arrival_time, depart_time, nb_bike_to_pick, nb_bike_to_drop, location):
        self.__location = location
        self.__arrival_time = arrival_time
        self.__depart_time = depart_time
        self.__nb_bike_to_pick = nb_bike_to_pick
        self.__nb_bike_to_drop = nb_bike_to_drop
        self.__nb_picked_bikes = 0  # Assuming no bikes have been picked up initially
        self.__nb_dropped_bikes = 0  # Assuming no bikes have been dropped off initially

    @property
    def location(self):
        """int: Retrieves the location identifier of the stop."""
        return self.__location

    @location.setter
    def location(self, value):
        self.__location= value

    @property
    def arrival_time(self):
        """float: Retrieves the arrival time at the stop."""
        return self.__arrival_time

    @arrival_time.setter
    def arrival_time(self, value):
        self.__arrival_time = value

    @property
    def depart_time(self):
        """float: Retrieves the departure time from the stop."""
        return self.__depart_time

    @depart_time.setter
    def depart_time(self, value):
        self.__depart_time = value

    @property
    def nb_bike_to_pick(self):
        """int: Retrieves the number of bikes intended to be picked up at the stop."""
        return self.__nb_bike_to_pick

    @nb_bike_to_pick.setter
    def nb_bike_to_pick(self, value):
        self.__nb_bike_to_pick = value

    @property
    def nb_bike_to_drop(self):
        """int: Retrieves the number of bikes intended to be dropped off at the stop."""
        return self.__nb_bike_to_drop

    @nb_bike_to_drop.setter
    def nb_bike_to_drop(self, value):
        self.__nb_bike_to_drop = value

    @property
    def nb_picked_bikes(self):
        """int: Retrieves the number of bikes actually picked up."""
        return self.__nb_picked_bikes

    @nb_picked_bikes.setter
    def nb_picked_bikes(self, value):
        self.__nb_picked_bikes = value

    @property
    def nb_dropped_bikes(self):
        """int: Retrieves the number of bikes actually dropped off."""
        return self.__nb_dropped_bikes

    @nb_dropped_bikes.setter
    def nb_dropped_bikes(self, value):
        self.__nb_dropped_bikes = value


    def __str__(self):
        class_string = str(self.__class__) + ": {"
        for attribute, value in self.__dict__.items():
            if "__nb_bike_to_pick" in attribute:
                class_string += str(attribute) + ": " \
                                + str(list(str(x.id) for x in value)) + ", "
            elif "__nb_picked_bikes" in attribute:
                class_string += str(attribute) + ": " \
                                + str(list(str(x.id) for x in value)) + ", "
            elif "__nb_bike_to_drop" in attribute:
                class_string += str(attribute) + ": " \
                                + str(list(str(x.id) for x in value)) + ", "
            elif "__nb_dropped_bikes" in attribute:
                class_string += str(attribute) + ": " \
                                + str(list(str(x.id) for x in value)) + ", "
            else:
                class_string += str(attribute) + ": " + str(value) + ", "

        class_string += "}"

        return class_string




class Location:
    """
    Represents a location with an integer label and optional longitude and latitude coordinates.
    This class is designed to store and manipulate basic information about the location of stations

    Attributes
    ----------
    label : int
        An integer label identifying the location id.
    lon : float, optional
        The longitude of the location.
    lat : float, optional
        The latitude of the location.
    """

    def __init__(self, label, lon=None, lat=None):
        """
        Initializes a new Location instance.

        Parameters
        ----------
        label : int
            An integer label for the location.
        lon : float, optional
            The longitude of the location (default is None).
        lat : float, optional
            The latitude of the location (default is None).
        """
        self.label = label
        self.lon = lon
        self.lat = lat

    def __str__(self):
        """
        Returns a string representation of the Location instance, including its label and,
        if available, its longitude and latitude coordinates.

        Returns
        -------
        str
            A string describing the location.
        """
        if self.lon is not None or self.lat is not None:
            ret_str = "{}: ({},{})".format(self.label, self.lon, self.lat)
        else:
            ret_str = "{}".format(self.label)

        return ret_str

    def __eq__(self, other):
        """
        Determines whether two Location instances are equal based on their labels.

        Parameters
        ----------
        other : Location
            Another Location instance to compare with.

        Returns
        -------
        bool
            True if the labels are the same, False otherwise.
        """
        if isinstance(other, Location):
            return self.label == other.label
        return False

    def __deepcopy__(self, memo):
        """
        Creates a deep copy of the Location instance.

        Parameters
        ----------
        memo : dict
            A dictionary of already copied objects to prevent recursive copying.

        Returns
        -------
        Location
            A new instance that is a deep copy of this instance.
        """
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result
