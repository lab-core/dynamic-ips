import requests

def get_path(origin, destination):
    """
        Get the shortest path between two points using OSRM API.

        Parameters:
        origin (tuple): A tuple containing the latitude and longitude coordinates of the origin point.
        destination (tuple): A tuple containing the latitude and longitude coordinates of the destination point.

        Returns:
        dict: A dictionary containing information about the shortest path, including coordinates, distance, and duration.

        Note:
        - This function utilizes the OSRM (Open Source Routing Machine) API to calculate the shortest path.
        - The input coordinates should be in the WGS 84 coordinate system (EPSG:4326), i.e., latitude and longitude format.
        - The returned dictionary includes the following keys:
          - 'origin_lat': Latitude of the origin point.
          - 'origin_lon': Longitude of the origin point.
          - 'destination_lat': Latitude of the destination point.
          - 'destination_lon': Longitude of the destination point.
          - 'distance': A list of distances (in meters) for each segment of the path.
          - 'duration': A list of durations (in seconds) for each segment of the path.
    """
    origin_lat, origin_lon = origin
    destination_lat, destination_lon = destination

    loc = "{},{};{},{}".format(origin_lon, origin_lat, destination_lon, destination_lat)
    url = "https://router.project-osrm.org/route/v1/driving/"
    option = "?generate_hints=false&annotations=duration,distance"
    r = requests.get(url + loc + option)
    if r.status_code != 200:
        return {}

    res = r.json()

    distance = res['distances']
    duration = res['durations']
    out = {
        'origin_lat': origin_lat,
        'origin_lon': origin_lon,
        'destination_lat': destination_lat,
        'destination_lon': destination_lon,
        'distance': distance,
        'duration': duration
        }

    return out