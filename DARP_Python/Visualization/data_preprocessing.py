"""
Data preprocessing module for extracting and flattening nested CSV data
Handles request, vehicle, and time data types
"""

import os
import pandas as pd
from typing import Literal, Optional, Callable, Dict, List, Any, Union
from functools import reduce
from Simulation.utilities import read_csv_with_encoding
import constants as c

AGG_FUNCS = {
    "mean": pd.Series.mean,
    "sum": pd.Series.sum,
    "median": pd.Series.median,
    "std": pd.Series.std,
    "min": pd.Series.min,
    "max": pd.Series.max,
    "count": lambda s: s.shape[0],
}



def preprocess_nested_data(
        df: pd.DataFrame,
        data_path: str,
        data_type: Literal['request', 'vehicle', 'time', None],
        value_column: Union[str, List[str]],
        instance_column: str = 'Instance',
        algorithm_column: str = 'Algorithm',
        filter_condition: Optional[Callable] = None,
        transform_func: Optional[Callable] = None,
        aggregate_func: Optional[str] = None,
        additional_columns: Optional[List[str]] = None,
) -> pd.DataFrame:

    base_path = os.path.dirname(data_path)

    preserve_columns = ['Ride_W2'] if 'Ride_W2' in df.columns else []
    if additional_columns:
        preserve_columns.extend(additional_columns)

    rows = []
    aggregated_rows = []

    def build_file_path(row):
        mode = f"{c.MODE_DICT.get(row['Mode'])}_{row[algorithm_column]}"
        file_name = c.DATA_TYPE_FILES[data_type].format(mode=mode)
        return os.path.join(base_path, row['Test_Folder'], file_name)

    # Iterate through main dataframe entries
    for _, row in df.iterrows():
        file_path = build_file_path(row)

        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            continue

        try:
            nested_data = read_csv_with_encoding(file_path)

            # Filter nested data if needed
            if filter_condition is not None:
                nested_data = filter_condition(nested_data)
                if nested_data.empty:
                    continue
                nested_data = nested_data.reset_index(drop=True)

            # Ensure value_column is a list
            columns_to_extract = value_column if isinstance(value_column, list) else [value_column]

            # Keep only existing columns
            existing_columns = [col for col in columns_to_extract if col in nested_data.columns]
            if not existing_columns:
                print(f"Warning: None of {columns_to_extract} found in {file_path}")
                continue

            instance_id = row[instance_column]

            # -----------------------------------
            # 🔹 AGGREGATED MODE
            # -----------------------------------
            if aggregate_func is not None:

                # Row that will hold all aggregated values
                agg_row = {
                    instance_column: instance_id,
                    algorithm_column: row[algorithm_column],
                    "Algorithm_Name": row["Algorithm_Name"],
                }

                # Preserve additional columns
                agg_row.update({p: row[p] for p in preserve_columns if p in row})

                # Aggregate each requested column
                for col in existing_columns:
                    values = nested_data[col]

                    if transform_func is not None:
                        values = transform_func(values)

                    func = AGG_FUNCS.get(aggregate_func, pd.Series.mean)
                    agg_row[col] = func(values)

                aggregated_rows.append(agg_row)

            # -----------------------------------
            # 🔹 NON-AGGREGATED MODE
            # -----------------------------------
            else:
                for i in range(len(nested_data)):
                    row_dict = {
                        instance_column: instance_id,
                        algorithm_column: row[algorithm_column],
                    }

                    # Preserve extra columns
                    row_dict.update({p: row[p] for p in preserve_columns if p in row})

                    # Add values for *all* columns at this row
                    for col in existing_columns:
                        v = nested_data[col].iloc[i]
                        if transform_func:
                            v = transform_func(v)
                        row_dict[col] = v

                    rows.append(row_dict)

        except Exception as e:
            print(f"Error reading {file_path}: {e}")

    # Return final dataframe
    return pd.DataFrame(aggregated_rows if aggregate_func else rows)




def extract_all_columns_for_data_type(
        df: pd.DataFrame,
        data_path: str,
        data_type: Literal['request', 'vehicle', 'time'],
        instance_column: str = 'Instance',
        algorithm_column: str = 'Algorithm',
        filter_condition: Optional[Callable] = None,
        aggregate_func: Optional[str] = 'mean',
        **kwargs
) -> pd.DataFrame:

    columns = c.DATA_TYPE_COLUMNS.get(data_type)
    if not columns:
        raise ValueError(f"Unknown data_type: {data_type}")

    dfs = []
    for col in columns:
        try:
            sub = preprocess_nested_data(
                df=df,
                data_path=data_path,
                data_type=data_type,
                value_column=col,
                instance_column=instance_column,
                algorithm_column=algorithm_column,
                filter_condition=filter_condition,
                aggregate_func=aggregate_func,
                **kwargs
            )
            if not sub.empty:
                dfs.append(sub)
        except Exception as e:
            print(f"Error extracting {col}: {e}")

    if not dfs:
        return pd.DataFrame()

    # concat on index columns instead of repeated merges
    dfs = [df.set_index([instance_column, algorithm_column]) for df in dfs]
    out = pd.concat(dfs, axis=1).reset_index()

    return out


