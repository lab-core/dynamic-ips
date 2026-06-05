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

# Maps current algorithm names back to their legacy equivalents used in old output filenames.
_ALGO_FILE_FALLBACK = {'B_CG': 'RT_CG', 'F_ICG': 'MP_ISUD'}


def preprocess_nested_data(
        df: pd.DataFrame,
        data_path: str,
        data_type: Literal['request', 'vehicle', 'time', 'epochVehicle', None],
        value_column: Union[str, List[str]],
        instance_column: str = 'Instance',
        algorithm_column: str = 'Algorithm',
        filter_condition: Optional[Callable] = None,
        transform_func: Optional[Callable] = None,
        aggregate_func: Optional[Union[str, List[str]]] = None,
        additional_columns: Optional[List[str]] = None,
) -> pd.DataFrame:

    # Supported aggregation functions
    AGG_FUNCS = {
        "mean": pd.Series.mean,
        "sum": pd.Series.sum,
        "median": pd.Series.median,
        "std": pd.Series.std,
        "min": pd.Series.min,
        "max": pd.Series.max,
        "count": lambda s: s.shape[0],
        "q25": lambda s: s.quantile(0.25),
        "q75": lambda s: s.quantile(0.75),
    }

    base_path = os.path.dirname(data_path)

    # Additional preserved columns
    preserve_columns = ['Ride_W2'] if 'Ride_W2' in df.columns else []
    if additional_columns:
        preserve_columns.extend(additional_columns)

    rows = []
    aggregated_rows = []

    def build_file_path(row):
        algo = row[algorithm_column]
        mode = f"{c.MODE_DICT.get(row['Mode'])}_{algo}"
        file_name = c.DATA_TYPE_FILES[data_type].format(mode=mode)
        path = os.path.join(base_path, row['Test_Folder'], file_name)
        if not os.path.exists(path) and algo in _ALGO_FILE_FALLBACK:
            legacy_mode = f"{c.MODE_DICT.get(row['Mode'])}_{_ALGO_FILE_FALLBACK[algo]}"
            legacy_name = c.DATA_TYPE_FILES[data_type].format(mode=legacy_mode)
            legacy_path = os.path.join(base_path, row['Test_Folder'], legacy_name)
            if os.path.exists(legacy_path):
                return legacy_path
        return path

    # ---------------------------------------------------------------
    # Traverse parent CSV rows
    # ---------------------------------------------------------------
    for _, row in df.iterrows():

        file_path = build_file_path(row)

        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            continue

        try:
            nested_data = read_csv_with_encoding(file_path)

            # Apply custom filtering
            if filter_condition is not None:
                nested_data = filter_condition(nested_data)
                if nested_data.empty:
                    continue
                nested_data = nested_data.reset_index(drop=True)

            # Ensure value_column is list
            columns_to_extract = value_column if isinstance(value_column, list) else [value_column]
            existing_columns = [col for col in columns_to_extract if col in nested_data.columns]

            if not existing_columns:
                print(f"Warning: none of {columns_to_extract} in {file_path}")
                continue

            instance_id = row[instance_column]

            # ===============================================================
            # 🔹 SPECIAL AGGREGATION — EPOCH VEHICLE DATA
            # ===============================================================
            if data_type == "epochVehicle" and aggregate_func is not None:

                # Normalize aggregate_func to list
                funcs = aggregate_func if isinstance(aggregate_func, list) else [aggregate_func]

                # Validate funcs
                for f in funcs:
                    if f not in AGG_FUNCS:
                        raise ValueError(f"Unknown aggregate_func '{f}'")

                if "Epoch" not in nested_data.columns:
                    print(f"Warning: 'Epoch' not found in {file_path}")
                    continue

                # Group by Epoch → each group is all vehicles at that epoch
                epoch_groups = nested_data.groupby("Epoch")

                for epoch_value, group in epoch_groups:

                    agg_row = {
                        instance_column: instance_id,
                        algorithm_column: row[algorithm_column],
                        "Algorithm_Name": row["Algorithm_Name"],
                        "Epoch": epoch_value,
                    }

                    # Append preserved columns
                    for p in preserve_columns:
                        if p in row:
                            agg_row[p] = row[p]

                    # Apply each aggregation function per column
                    for func_name in funcs:
                        func = AGG_FUNCS[func_name]
                        for col in existing_columns:
                            values = group[col]
                            if transform_func:
                                values = transform_func(values)
                            agg_row[f"{func_name}_{col}"] = func(values)

                    aggregated_rows.append(agg_row)

                continue  # Important: skip other logic

            # ===============================================================
            # 🔹 DEFAULT AGGREGATION (for other data types)
            # ===============================================================
            if aggregate_func is not None:

                funcs = aggregate_func if isinstance(aggregate_func, list) else [aggregate_func]

                for f in funcs:
                    if f not in AGG_FUNCS:
                        raise ValueError(f"Unknown aggregate_func '{f}'")

                agg_row = {
                    instance_column: instance_id,
                    algorithm_column: row[algorithm_column],
                    "Algorithm_Name": row["Algorithm_Name"],
                }

                # Preserve columns
                for p in preserve_columns:
                    if p in row:
                        agg_row[p] = row[p]

                # Compute aggregation per column
                for func_name in funcs:
                    func = AGG_FUNCS[func_name]

                    for col in existing_columns:
                        values = nested_data[col]
                        if transform_func:
                            values = transform_func(values)
                        agg_row[f"{func_name}_{col}"] = func(values)

                aggregated_rows.append(agg_row)
                continue

            # ===============================================================
            # 🔹 NON-AGGREGATED MODE — return full expanded rows
            # ===============================================================
            chunk = nested_data[existing_columns].copy()

            if transform_func:
                for col in existing_columns:
                    chunk[col] = chunk[col].map(transform_func)

            chunk[instance_column] = instance_id
            chunk[algorithm_column] = row[algorithm_column]

            for p in preserve_columns:
                if p in row:
                    chunk[p] = row[p]

            rows.append(chunk)

        except Exception as e:
            print(f"Error reading {file_path}: {e}")

    # ===============================================================
    # RETURN FINAL OUTPUT
    # ===============================================================
    if aggregate_func:
        return pd.DataFrame(aggregated_rows)
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


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


from typing import List, Dict, Optional
import pandas as pd

def build_vehicle_kpi_long_df(
        nested_df: pd.DataFrame,
        value_columns: List[str],
        kpi_name_map: Dict[str, str],
        category_column: str,
        item_column_name: str = "KPI",
        value_col_name: str = "value",
        extra_id_columns: Optional[List[str]] = None,
) -> pd.DataFrame:
    """
    Take a WIDE nested_df (e.g. output of preprocess_nested_data with value_columns)
    and melt it to LONG form for boxplots.

    Input:
        nested_df: DataFrame that already contains:
            - the KPI columns in value_columns (e.g. 'idleTime', 'driveEmptyTime')
            - the category_column (e.g. 'paramFile', 'object_category', ...)
            - optionally extra id columns like 'Instance', 'Algorithm', etc.
        value_columns: list of KPI columns to melt.
        kpi_name_map: dict mapping column name -> display name.
        category_column: name of the grouping column (sticks around in long df).
        item_column_name: name of the “KPI” column in the long df.
        value_col_name: name of the “value” column in the long df.
        extra_id_columns: other id columns you want to keep (e.g. ['Instance', 'Algorithm']).

    Output columns:
        - item_column_name  (e.g. 'KPI')
        - category_column   (e.g. 'paramFile')
        - value_col_name    (e.g. 'value')
        - plus whatever you kept in extra_id_columns
    """
    if nested_df.empty:
        return nested_df.copy()

    # id variables that should not be melted
    id_vars = [category_column]
    if extra_id_columns:
        id_vars.extend(extra_id_columns)

    # only keep id_vars that actually exist
    id_vars = [c for c in id_vars if c in nested_df.columns]

    long_df = nested_df.melt(
        id_vars=id_vars,
        value_vars=value_columns,
        var_name=item_column_name,
        value_name=value_col_name,
    )

    # Map raw column names to friendly labels
    long_df[item_column_name] = long_df[item_column_name].map(
        lambda col: kpi_name_map.get(col, col)
    )

    return long_df
