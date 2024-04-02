from src.simulator.task import Task


class Station:
    """
    Represents a station in the transportation network, encapsulating details like its
    identifier, name, location, capacity, and the sector it belongs to, along with any
    current task assigned to it.

    Attributes
    ----------
    station_id : int
        Unique identifier for the station.
    short_name : str
        Shortened name of the station.
    name : str
        Full name of the station.
    location : Location
        Identifier for the station's location.
    capacity : int
        Capacity of the station (could be in terms of bikes or vehicles it can accommodate).
    sector : int
        Sector where the station is located.
    current_task : Task
        The task currently assigned to the station, if any.
    """

    def __init__(self, station_id, short_name, name, location, capacity, sector):
        self.__station_id = station_id
        self.__short_name = short_name
        self.__name = name
        self.__location = location
        self.__capacity = capacity
        self.__sector = sector
        self.__current_task = Task(station=self)

    @property
    def station_id(self):
        return self.__station_id

    @property
    def short_name(self):
        return self.__short_name

    @property
    def name(self):
        return self.__name

    @property
    def location(self):
        return self.__location

    @property
    def capacity(self):
        return self.__capacity

    @property
    def sector(self):
        return self.__sector

    @property
    def current_task(self):
        return self.__current_task

    @station_id.setter
    def station_id(self, value):
        self.__station_id = value

    @short_name.setter
    def short_name(self, value):
        self.__short_name = value

    @name.setter
    def name(self, value):
        self.__name = value

    @location.setter
    def location(self, value):
        self.__location = value

    @capacity.setter
    def capacity(self, value):
        self.__capacity = value

    @sector.setter
    def sector(self, value):
        self.__sector = value

    @current_task.setter
    def current_task(self, value):
        self.__current_task = value

    def __str__(self):
        """
        Generates a string representation of the Station object, including all attributes.
        """
        class_string = f"{self.__class__}: {{"
        for attribute, value in self.__dict__.items():
            attr_name = attribute.replace("_Station__", "")
            class_string += f"{attr_name}: {value}, "
        class_string = class_string.rstrip(", ") + "}"
        return class_string


