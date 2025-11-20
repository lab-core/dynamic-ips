import os
from typing import Literal

from matplotlib import pyplot as plt
from matplotlib.figure import Figure

import constants as c
import datetime as dtime
import pandas as pd
import matplotlib.colors as mc

def create_dates():
    file_names = []
    for item in c.DATES_2015:
        file_names.append("2015-" + item)
    for item in c.DATES_2016:
        file_names.append("2016-" + item)
    return file_names


def create_file_names():
    file_names = []
    for item in c.DATES_2015:
        file_names.append("2015-" + item + "_manhattan")
    for item in c.DATES_2016:
        file_names.append("2016-" + item + "_manhattan")
    return file_names

def calculate_time_from_origin_sec(time_origin, start_hr, end_hr, start_min, end_min):
    period_start, period_end = calculate_time_from_origin(time_origin, start_hr, end_hr, start_min, end_min)
    start_seconds = (period_start - time_origin).total_seconds()
    end_seconds = (period_end - time_origin).total_seconds()
    return start_seconds, end_seconds

def calculate_time_from_origin(time_origin, start_hr, end_hr, start_min, end_min):
    year = time_origin.year
    month = time_origin.month
    day = time_origin.day
    period_start = dtime.datetime(year, month, day, start_hr, start_min, 0)
    period_end = dtime.datetime(year, month, day, end_hr, end_min, 0)
    return period_start, period_end

def get_instance_value(name, mapping):
    for key in mapping.keys():
        if name.startswith(key):
            return mapping[key]
    return name

def get_group(size):
    for key, value in c.TEST_SIZE.items():
        if value[0] <= size <= value[1]:
            return c.TEST_GROUP[key]

def read_csv_with_encoding(file_path):
    """Try to read CSV with multiple encodings."""
    encodings = ['utf-8', 'latin1', 'iso-8859-1', 'cp1252']
    for encoding in encodings:
        try:
            return pd.read_csv(file_path, encoding=encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("Unable to read the file with provided encodings.")

def darken_color(color, factor=0.8):
    """
    Darkens a given color by the specified factor.
    Factor < 1 will darken the color, and Factor > 1 will lighten it.
    """
    rgb = mc.to_rgb(color)  # Convert the color to RGB
    return tuple([c * factor for c in rgb])
