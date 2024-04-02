class Solver_Parameters:
    """
    Encapsulates selected model parameters for optimization and solution approaches.

    Attributes
    ----------
    epoch_length : int
        Epoch length for re-optimization.
    nb_threads : int
        Number of threads for parallel optimization.
    main_algorithm : MainAlgorithm
        The algorithm used for solving the master problem.
    nb_column : int
        Number of columns to be added in column generation.
    is_truncated : bool
        If a truncated labeling strategy is used.
    max_label : int
        Maximum number of labels to keep while using truncated labeling.
    is_drop_pick_possible : bool
        If picking up after dropping is allowed as an acceleration strategy.
    labeling_strategy : LabelingStrategy
        Labeling strategy used.
    nb_pick : int
        Number of pickups allowed.
    sort_path : SortPaths
        Sorting mode of paths.
    mip_gap : float
        MIP gap used as a stopping criterion.
    """

    def __init__(self, epoch_length, nb_threads, main_algorithm, nb_column, is_truncated,
                 max_label, is_drop_pick_possible, labeling_strategy, nb_pick, sort_path, mip_gap):
        self.epoch_length = epoch_length
        self.nb_threads = nb_threads
        self.main_algorithm = main_algorithm
        self.nb_column = nb_column
        self.is_truncated = is_truncated
        self.max_label = max_label
        self.is_drop_pick_possible = is_drop_pick_possible
        self.labeling_strategy = labeling_strategy
        self.nb_pick = nb_pick
        self.sort_path = sort_path
        self.mip_gap = mip_gap

    def __str__(self):
        """
        Generates a string representation of the Parameters object, detailing all relevant attributes dynamically.
        """
        class_string = f"{self.__class__.__name__}: {{"
        for attribute, value in self.__dict__.items():
            attr_name = attribute.lstrip('_')
            class_string += f"{attr_name}: {value}, "
        class_string = class_string.rstrip(', ') + "}"
        return class_string

class Parameters:
    """
    Encapsulates simulation parameters for a transportation network.

    Attributes
    ----------
    simulation_start : int
        The start time of the simulation in seconds since the beginning of the day.
    num_vehicles : int
        The number of vehicles available in the simulation.
    num_stations : int
        The number of stations in the simulation.
    time_per_bike : float
        The average time (in seconds) it takes to handle each bike.
    time_to_park : int
        The time (in seconds) it takes to park a vehicle at a station.
    """

    def __init__(self, simulation_start, num_vehicles, num_stations, time_per_bike, time_to_park):
        self.simulation_start = simulation_start
        self.num_vehicles = num_vehicles
        self.num_stations = num_stations
        self.time_per_bike = time_per_bike
        self.time_to_park = time_to_park

    def __str__(self):
        """
        Generates a string representation of the SimulationParameters object.
        """
        return (f"SimulationParameters(simulation_start={self.simulation_start}, "
                f"num_vehicles={self.num_vehicles}, num_stations={self.num_stations}, "
                f"time_per_bike={self.time_per_bike}, time_to_park={self.time_to_park})")

