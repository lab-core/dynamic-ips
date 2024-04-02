from enum import Enum
from datetime import datetime

class TaskStatus(Enum):
    """
    Task statuses:
    - INITIALIZED: The task is only initialized and has no transfer value
    - RELEASED : The task is made available in the system but has not yet been assigned.
    - UPDATED : The task details have been modified after its initial release.
    - DISPATCHED : The task has been assigned and is currently being executed.
    """
    INITIALIZED = "initialized"
    RELEASED = "released"
    UPDATED = "updated"
    DISPATCHED = "dispatched"

class Task:
    """
    Represents a re-balancing task in the system.

    Attributes
    ----------
    station : Station
        Unique identifier for the station.
    nb_transfer : int
        Number of bikes to transfer at the station.
    relocate_cost : float
        Relocate cost of performing the task.
    dual : float
        The dual value of the task from the MP.
    request_time : datetime
        The time when the task was requested, in datetime format.
    request_time_seconds : int
        The time when the task was requested, in seconds from the specified base_time.
    task_status : TaskStatus
        The current status of the task.
    perform_time : datetime
        The time when the task started to be performed.
    """

    def __init__(self, station):
        self.__station = station

        self.__request_time = None
        self.__task_status = TaskStatus.INITIALIZED
        self.__request_time_seconds = 0
        self.__perform_time = None  # Initialize as None, assuming the task has not been performed yet
        self.__nb_transfer = 0
        self.__relocate_cost = 0
        self.__dual = 0

    def release(self, nb_transfer, relocate_cost, request_time,
                base_time=datetime.now().replace(hour=7, minute=0, second=0, microsecond=0)):
        """
        Update the initial task after release with new values and records the update time.

        Parameters:
        - nb_transfer (int): Optional. The new number of bikes to transfer.
        - relocate_cost (float): Optional. The new relocate cost.
        - task_status (TaskStatus): Optional. The new task status.
        - request_time (datetime): Optional. The new request time.
        """
        self.__nb_transfer = nb_transfer
        self.__relocate_cost = relocate_cost
        self.__task_status = TaskStatus.RELEASED
        self.__request_time = request_time
        self.__request_time_seconds = int((request_time - base_time).total_seconds())

    def update(self, nb_transfer, relocate_cost, update_time, base_time):
        """
            Update the available task in the system.

            Parameters:
            - nb_transfer (int): Optional. The new number of bikes to transfer.
            - relocate_cost (float): Optional. The new relocate cost.
            - task_status (TaskStatus): Optional. The new task status.
            - request_time (datetime): Optional. The new request time.
            """
        self.__nb_transfer = nb_transfer
        self.__relocate_cost = relocate_cost
        self.__task_status = TaskStatus.UPDATED
        self.__request_time = update_time
        self.__request_time_seconds = int((update_time - base_time).total_seconds())

    @property
    def station(self):
        return self.__station

    @station.setter
    def station(self, value):
        self.__station = value

    @property
    def nb_transfer(self):
        return self.__nb_transfer

    @nb_transfer.setter
    def nb_transfer(self, value):
        self.__nb_transfer = value

    @property
    def relocate_cost(self):
        return self.__relocate_cost

    @relocate_cost.setter
    def relocate_cost(self, value):
        self.__relocate_cost = value

    @property
    def dual(self):
        return self.__dual

    @dual.setter
    def dual(self, value):
        self.__dual = value

    @property
    def request_time(self):
        return self.__request_time

    @property
    def request_time_seconds(self):
        return self.__request_time_seconds

    @property
    def task_status(self):
        return self.__task_status

    @task_status.setter
    def task_status(self, value):
        self.__task_status = value


    @property
    def perform_time(self):
        return self.__perform_time

    @perform_time.setter
    def perform_time(self, value):
        self.__perform_time = value

    def __str__(self):
        """
        Generates a string representation of the Task object, including all attributes.
        """
        class_string = f"{self.__class__.__name__}: {{"
        for attribute, value in self.__dict__.items():
            if attribute in ["_Task__station"] and hasattr(value, '__str__'):
                value_str = value.__str__()
            elif isinstance(value, datetime):
                value_str = value.strftime("%Y-%m-%d %H:%M:%S")
            elif isinstance(value, Enum):
                value_str = value.value
            else:
                value_str = str(value)
            attr_name = attribute.replace("_Task__", "")
            class_string += f"{attr_name}: {value_str}, "
        class_string = class_string.rstrip(", ") + "}"
        return class_string
