from Simulation.transform_day_inputs import _build_stop_index, transform_multiple_files
import Simulation.utilities as ut
from Simulation.get_trip_records import fetch_data_for_date
from Simulation.transform_day_inputs import _build_stop_index, transform_multiple_files, nearest_stop

def main_get_days():
    for date in ut.create_dates():
        fetch_data_for_date(date_str=date)

def main_transform():
    try:
        # You need to build the stop index first
        # This requires your virtual_stops_latlon.geojson file
        print("Building stop index...")
        stop_index = _build_stop_index()
     #   result = nearest_stop(lat=40.644832611083984, lon=-73.78178405761719,index=stop_index,)
        print("Stop index built successfully!")

        # Transform a single file
        # transform_json_file('taxi_2015.json', 'transformed_2015.json', 2015, stop_index)
        # transform_json_file('taxi_2016.json', 'transformed_2016.json', 2016, stop_index)

        # Or transform multiple files
        transform_multiple_files('Data/days/', 'Data/transform_days/', stop_index)

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Make sure you have the virtual_stops_latlon.geojson file in the correct location.")
        print("You may need to run create_virtual_stops() first.")


# Use scripts/02_fetch_trips.py and scripts/03_transform_trips.py for CLI:
#   python scripts/02_fetch_trips.py
#   python scripts/03_transform_trips.py --network own
if __name__ == "__main__":
    main_transform()


