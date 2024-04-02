import logging
import copy

logger = logging.getLogger(__name__)


class Vehicle:
    """
    The Vehicle class encapsulates information pertinent to trucks

    Attributes
    ----------
    vehicle_id : int
        Unique identifier for the vehicle.
    start_time : float
        Operational start time for the vehicle.
    end_time : float
        Operational end time for the vehicle.
    depart_stop : Stop
        Departing stop of the vehicle.
    capacity : int
        Maximum passenger capacity of the vehicle.
    bike_load : int
        Capacity for bike transport.
    current_route : Route_Plan
        Current route plan for the vehicle.
    solution_route : Route
        complete route of the vehicle
    dual : float
        Dual value associated with the vehicle in the optimization model.

    """

    def __init__(self, vehicle_id, start_time, end_time, depart_stop, capacity, bike_load):
        self.vehicle_id = vehicle_id
        self.start_time = start_time
        self.end_time = end_time
        self.depart_stop = depart_stop
        self.capacity = capacity
        self.bike_load = bike_load
        self.current_route = None
        self.solution_route = None
        self.dual = 0.0

    # Property decorators for encapsulating access to the vehicle's attributes.
    @property
    def vehicle_id(self):
        """int: Retrieves the unique identifier of the vehicle."""
        return self.__vehicle_id

    @vehicle_id.setter
    def vehicle_id(self, value):
        self.__vehicle_id = value

    @property
    def start_time(self):
        """float: Retrieves the operational start time of the vehicle."""
        return self.__start_time

    @start_time.setter
    def start_time(self, value):
        self.__start_time = value

    @property
    def end_time(self):
        """float: Retrieves the operational end time of the vehicle."""
        return self.__end_time

    @end_time.setter
    def end_time(self, value):
        self.__end_time = value

    @property
    def depart_stop(self):
        """PStop: Retrieves the departing stop of the vehicle."""
        return self.__depart_stop

    @depart_stop.setter
    def depart_stop(self, value):
        self.__depart_stop = value

    @property
    def capacity(self):
        """int: Retrieves the maximum passenger capacity of the vehicle."""
        return self.__capacity

    @capacity.setter
    def capacity(self, value):
        self.__capacity = value

    @property
    def bike_load(self):
        """int: Retrieves the bike transport capacity of the vehicle."""
        return self.__bike_load

    @bike_load.setter
    def bike_load(self, value):
        self.__bike_load = value

    @property
    def current_route(self):
        """PRoutePlan: Retrieves the current route plan of the vehicle."""
        return self.__current_route

    @current_route.setter
    def current_route(self, value):
        self.__current_route = value

    @property
    def solution_route(self):
        """PRoute: Retrieves the executed route of the vehicle."""
        return self.__solution_route

    @solution_route.setter
    def solution_route(self, value):
        self.__solution_route = value

    @property
    def dual(self):
        """float: Retrieves the dual value associated with the vehicle's route."""
        return self.__dual

    @dual.setter
    def dual(self, value):
        self.__dual = value

