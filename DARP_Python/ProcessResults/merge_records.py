from pathlib import Path
from typing import Optional, Literal
import pandas as pd
from tqdm import tqdm
import constants as c
from Simulation.utilities import get_instance_value, get_group


class ResultMerger:
    """Configurable merger for optimization results with flexible data inclusion."""

    def __init__(self, instance_folder: str):
        self.root_folder = Path(c.OUTPUT_DIR) / instance_folder
        self.mode_dict = {'ANYTIME': 'A', 'DYNAMIC': 'D'}
        self.algo_map = {
            ('GREEDY', 'ANYTIME', 'FALSE','FALSE'): 'Greedy',
            ('GREEDY', 'ANYTIME', 'TRUE', 'FALSE'): 'Greedy-R',
            ('GREEDY', 'ANYTIME', 'TRUE', 'TRUE'): 'Greedy-CR',
            ('RT_CG', 'DYNAMIC', 'FALSE', 'FALSE'): 'B-CG',
            ('RT_CG', 'DYNAMIC', 'FALSE', 'TRUE'): 'B-CG-C',
            ('RT_CG', 'DYNAMIC', 'TRUE', 'TRUE'): 'B-CG-CR',
            ('A_CG', 'ANYTIME', 'FALSE', 'TRUE'): 'A-CG-C',
            ('A_CG', 'ANYTIME', 'TRUE', 'TRUE'): 'A-CG-CR',
            ('MP_ISUD', 'DYNAMIC', 'FALSE', 'FALSE'): 'B-ICG',
            ('MP_ISUD', 'DYNAMIC', 'FALSE', 'TRUE'): 'B-ICG-C',
            ('MP_ISUD', 'DYNAMIC', 'TRUE', 'TRUE'): 'B-ICG-CR',
            ('MP_ISUD', 'ANYTIME', 'FALSE', 'FALSE'): 'A-ICG',
            ('MP_ISUD', 'ANYTIME', 'FALSE', 'TRUE'): 'A-ICG-C',
            ('MP_ISUD', 'ANYTIME', 'TRUE', 'TRUE'): 'A-ICG-CR',
        }

    def _get_file_suffix(self, data: pd.DataFrame) -> str:
        """Generate file suffix based on mode and algorithm."""
        mode = self.mode_dict[data['Mode'].iloc[-1]]
        alg = data['Algorithm'].iloc[-1]
        return f"{mode}_{alg}"

    def _find_file(self, directory: Path, prefix: str, suffix: str) -> Optional[Path]:
        """Find file matching pattern prefix_suffix.csv"""
        pattern = f"{prefix}_{suffix}.csv"
        file_path = directory / pattern
        return file_path if file_path.exists() else None

    def _read_summary_and_params(self, summary_path: Path) -> tuple[pd.DataFrame, pd.DataFrame, str]:
        """Read summary and parameter files, return data and suffix."""
        data = pd.read_csv(summary_path, index_col=False)
        suffix = self._get_file_suffix(data)

        # Set specific columns to zero if Algorithm is not MP_ISUD
        if 'Algorithm' in data.columns:
            isud_columns = ['#RP Iter', '#Zoom Iter', 'CPFails']
            for col in isud_columns:
                if col in data.columns:
                    data.loc[data['Algorithm'] != 'MP_ISUD', col] = 0

        # Modify Instance column based on test size
        if '#requests' in data.columns:
            if data['#requests'].iloc[0] < 2000:
                # Small test: use name_to_instance mapping
                if 'Name' in data.columns:
                    name_to_instance = c.name_to_instance
                    data['Instance'] = data['Name'].apply(
                        lambda name: get_instance_value(name, name_to_instance)
                    )
            else:
                # Large test: use #vehicles and #requests
                if '#vehicles' in data.columns:
                    data['Instance'] = data.apply(
                        lambda row: f"V{int(row['#vehicles'])}_R{int(row['#requests'])}",
                        axis=1
                    )
        # Modify group category
        data.loc[0, 'customer Group'] = get_group(data['#customers'][0])

        data['Instance_category'] = data['Instance'].map(c.NYC_DARP_Benchmark)


        # columns that need rounding
        round_cols_2 = [
            "wait/req", "wait/cust", "tripDelay/req", "tripDelay/cust",
            "idle time/vehicle", "#pass in vehicle",
            "MASTER time", "SP time", "Greedy time",
            "Rebalance time", "Total time"
        ]
        for col in round_cols_2:
            data[col] = data[col].round(2)

        round_cols_4 = [
            "MASTER/Total", "SP/Total"
        ]
        for col in round_cols_4:
            data[col] = data[col].round(4)


        par_path = self._find_file(summary_path.parent, "Parameters", suffix)
        if par_path:
            parameter_data = pd.read_csv(par_path, index_col=False)
            parameter_data = parameter_data.drop(columns=["Instance"], errors='ignore')

            # Add Committed column based on informTimeLimit
            if 'informTimeLimit' in parameter_data.columns:
                parameter_data['Committed'] = parameter_data['informTimeLimit'] < 300
            else:
                parameter_data['Committed'] = False

            # Add isSuccessorsLimited column based on pruning strategies
            parameter_data['isSuccessorsLimited'] = parameter_data.apply(
                lambda row: 'pruneNodes' if row['pruneNodes'] and not row['pruneArcs'] and not row[
                    'discardSuboptimalPath'] else
                'pruneArcs' if row['pruneNodes'] and row['pruneArcs'] and not row['discardSuboptimalPath'] else
                'discardSuboptimalPath' if row['pruneNodes'] and row['pruneArcs'] and row['discardSuboptimalPath'] else
                'noPruning',
                axis=1
            )
            parameter_data["object_category"] = ("wait = 1, delay = " + parameter_data["Ride_W2"].astype(str))
            if 'Relative_W5' in parameter_data.columns:
                parameter_data["object_category2"] = ("delay = " + parameter_data["Ride_W2"].astype(str) +
                                                      ", excess = " + parameter_data["Ride_W4"].astype(str) +
                                                      ", Relative = " + parameter_data["Relative_W5"].astype(str))
            parameter_data['isSuccessorsLimited'] = parameter_data['isSuccessorsLimited'].astype(str)
        else:
            print("Parameter file is not found!!!")
            parameter_data = pd.DataFrame()

        # Add folder information
        parent_and_current = str(Path(summary_path.parent.parent.name) / summary_path.parent.name)
        data['Test_Folder'] = parent_and_current

        return data, parameter_data, suffix

    def _add_min_max_stats(self, data: pd.DataFrame, summary_path: Path, suffix: str) -> pd.DataFrame:
        """Add min/max/avg statistics from request and vehicle data."""
        # Read request data
        req_path = self._find_file(summary_path.parent, "Requests", suffix)
        if req_path:
            request_data = pd.read_csv(req_path, index_col=False)
            # Filter unserved requests
            filtered_requests = request_data[request_data['TripDelay'] >= 0]
            request_data = filtered_requests.reset_index(drop=True)
            request_data['TripDelay'] = (request_data['DropTime'] - request_data['PickTime'] -
                                         request_data['MinTravelTime'] - 30)

            # Request metrics (optional: convert to minutes for time-based metrics)
            request_metrics = {
                'min_WaitTime': request_data['WaitTime'].min(),
                'max_WaitTime': request_data['WaitTime'].max(),
                'avg_WaitTime': request_data['WaitTime'].mean(),
                'min_CommitWait': request_data['CommitWaitTime'].min(),
                'max_CommitWait': request_data['CommitWaitTime'].max(),
                'avg_CommitWait': request_data['CommitWaitTime'].mean(),
                'min_TripDelay': request_data['TripDelay'].min(),
                'max_TripDelay': request_data['TripDelay'].max(),
                'avg_TripDelay': request_data['TripDelay'].mean(),
            }
            request_metrics = {k: round(v, 2) for k, v in request_metrics.items()}

            for col, val in request_metrics.items():
                data.at[0, col] = val

        # Read vehicle data
        veh_path = self._find_file(summary_path.parent, "Vehicles", suffix)
        if veh_path:
            vehicle_data = pd.read_csv(veh_path, index_col=False)

            vehicle_columns = ['#Stops', '#RequestsServed', 'idleTime', 'serviceTime',
                               'driveFullTime', 'driveEmptyTime', 'returnEmptyTime']

            for col in vehicle_columns:
                if col in vehicle_data.columns:
                    clean_col = col.replace('#', '')
                    data.at[0, f'min_{clean_col}'] = round(vehicle_data[col].min(), 2)
                    data.at[0, f'max_{clean_col}'] = round(vehicle_data[col].max(), 2)
                    data.at[0, f'avg_{clean_col}'] = round(vehicle_data[col].mean(), 2)

        return data

    def _add_epoch_data(self, data: pd.DataFrame, summary_path: Path, suffix: str,
                        average: bool = False) -> pd.DataFrame:
        """Add epoch runtime data, optionally averaged."""
        epoch_path = self._find_file(summary_path.parent, "epochRuntime", suffix)
        if not epoch_path:
            print("epochRuntime file is not found!!!")
            return pd.DataFrame()

        epoch_data = pd.read_csv(epoch_path, index_col=False)

        # Select relevant columns
        epoch_cols = ["EpochRuntime", "MP_Runtime", "SubProbRuntime", "Objective", "LinearObjective", "waitTime",
                      "TripDelay", "totalColumn", "#LGenerated", "#LDominated", "#LEliminated", "#nbPrunedArcs",
                      " #nbPrunedPath","nbNegative", "#ColumnsAdded", "totalRoutes", "#Return", "#Idle",
                      "#passPerVehicle", "#requestPerVehicle", "#nodePerVehicle",
                      "#StateChanged", "nbOnePick", "nbTwoPick", "nbThreePick", "meanDual"]

        # Filter to existing columns
        epoch_cols = [col for col in epoch_cols if col in epoch_data.columns]
        epoch_data = epoch_data[epoch_cols]

        if average:
            epoch_data = epoch_data.mean().to_frame().T

        round_cols = ["EpochRuntime", "MP_Runtime", "SubProbRuntime",
                      "#passPerVehicle", "#requestPerVehicle", "#nodePerVehicle", "meanDual"]
        for col in round_cols:
            epoch_data[col] = epoch_data[col].round(2)

        return epoch_data

    def _add_basic_epoch_data(
            self,
            data: pd.DataFrame,
            summary_path: Path,
            suffix: str,
            average: bool = False
    ) -> pd.DataFrame:
        """Add epoch runtime data, optionally averaged."""
        epoch_path = self._find_file(summary_path.parent, "epochRuntime", suffix)
        if not epoch_path:
            print("epochRuntime file is not found!!!")
            return pd.DataFrame()

        epoch_data = pd.read_csv(epoch_path, index_col=False)

        # Select relevant columns
        epoch_cols = [
            "EpochRuntime",
            "Objective",
            "waitTime",
            "TripDelay",
            "#passPerVehicle",
            "#requestPerVehicle",
            "#nodePerVehicle",
        ]

        # Filter to existing columns
        epoch_cols = [col for col in epoch_cols if col in epoch_data.columns]
        epoch_data = epoch_data[epoch_cols]

        # Compute mean row
        mean_data = epoch_data.mean().to_frame().T

        # Add std for EpochRuntime
        if "EpochRuntime" in epoch_data.columns:
            mean_data["EpochRuntime_std"] = epoch_data["EpochRuntime"].std()

        round_cols = [
            "EpochRuntime",
            "EpochRuntime_std",
            "#passPerVehicle",
            "#requestPerVehicle",
            "#nodePerVehicle"
        ]
        for col in round_cols:
            if col in mean_data.columns:
                mean_data[col] = mean_data[col].round(2)

        return mean_data

    def _add_iteration_data(
            self,
            data: pd.DataFrame,
            summary_path: Path,
            suffix: str,
            iteration_mode: Literal['none', 'all', 'max', 'max_limited']
    ) -> tuple[pd.DataFrame, pd.DataFrame]:
        """Add iteration-level data based on mode.

        Args:
            iteration_mode:
                - 'none': No iteration data
                - 'all': All iterations
                - 'max': Keep last LMP row and CG row for each (Epoch, subProIter)
                - 'max_limited': Same as 'max' but limited to subProIter <= 4
        """
        if iteration_mode == 'none':
            return data, pd.DataFrame()

        runtime_path = self._find_file(summary_path.parent, "epochResults", suffix)
        if not runtime_path:
            print("epochResults file is not found!!!")
            return data, pd.DataFrame()

        run_data = pd.read_csv(runtime_path, index_col=False)
        filtered_run_data = run_data.copy()

        # Optional limit
        if iteration_mode == 'max_limited' and 'subProIter' in filtered_run_data.columns:
            filtered_run_data = filtered_run_data[filtered_run_data['subProIter'] <= 4].copy()

        # For 'all', return everything unchanged except adding empty columns if desired
        if iteration_mode == 'all':
            if 'MIP_GAP' not in filtered_run_data.columns:
                filtered_run_data['MIP_GAP'] = pd.NA
            if 'IP_time' not in filtered_run_data.columns:
                filtered_run_data['IP_time'] = pd.NA

            num_iterations = len(filtered_run_data)
            data_repeated = pd.concat([data] * num_iterations, ignore_index=True)
            return data_repeated, filtered_run_data

        # For 'max' and 'max_limited':
        # keep last LMP row + CG row for each (Epoch, subProIter)
        group_cols = ['Epoch', 'subProIter'] if 'Epoch' in filtered_run_data.columns else ['subProIter']

        selected_groups = []

        for _, grp in filtered_run_data.groupby(group_cols, sort=False):
            grp = grp.sort_values('MPIter').copy() if 'MPIter' in grp.columns else grp.copy()

            lmp_rows = grp[grp['Model'] == 'LMP'] if 'Model' in grp.columns else pd.DataFrame()
            cg_rows = grp[grp['Model'] == 'CG'] if 'Model' in grp.columns else pd.DataFrame()
     #       rp_rows = grp[grp['Model'] == 'RP'] if 'Model' in grp.columns else pd.DataFrame()
            isud_rows = grp[grp['Model'] == 'ISUD'] if 'Model' in grp.columns else pd.DataFrame()

            # Keep the last LMP row
            last_lmp = lmp_rows.tail(1).copy() if not lmp_rows.empty else pd.DataFrame()
            first_row = grp.head(1).copy()

            # Keep the CG row (if multiple, keep the last one)
            cg_row = cg_rows.tail(1).copy() if not cg_rows.empty else pd.DataFrame()

            pair_parts = []
            if not last_lmp.empty:
                pair_parts.append(last_lmp)
            if not cg_row.empty:
                pair_parts.append(cg_row)
            if not isud_rows.empty:
                pair_parts.append(first_row)
                pair_parts.append(isud_rows)
                first_obj = first_row['preObjective'].iloc[0]
                final_obj = isud_rows['ObjectiveValue'].iloc[0]
                isud_rows['RelativeImprove'] = 100 * (first_obj-final_obj)/first_obj

            if not pair_parts:
                continue

            pair_df = pd.concat(pair_parts, ignore_index=False).copy()

            # Default values
            pair_df['MIP_GAP'] = pd.NA
            pair_df['IP_time'] = pd.NA
            pair_df['Iter_time'] = pd.NA

            # Compute values only if both LMP and CG exist
            if not last_lmp.empty and not cg_row.empty:
                lmp_obj = last_lmp['ObjectiveValue'].iloc[0]
                cg_obj = cg_row['ObjectiveValue'].iloc[0]

                lmp_mp_time = last_lmp['MPTimeAcc.'].iloc[0]
                cg_mp_time = cg_row['MPTimeAcc.'].iloc[0]

                cg_sp_time = cg_row['SubProTime'].iloc[0]

                # Absolute gap
                mip_gap = 100 * (cg_obj - lmp_obj)/lmp_obj

                # Time spent to get IP solution
                ip_time = cg_mp_time - lmp_mp_time
                iter_time = cg_sp_time + cg_mp_time

                pair_df['MIP_GAP'] = mip_gap
                pair_df['IP_time'] = ip_time
                pair_df['Iter_time'] = iter_time

            selected_groups.append(pair_df)

        if selected_groups:
            filtered_run_data = pd.concat(selected_groups, ignore_index=False)
            if 'MPIter' in filtered_run_data.columns:
                filtered_run_data = filtered_run_data.sort_values(group_cols + ['MPIter']).reset_index(drop=True)
            else:
                filtered_run_data = filtered_run_data.sort_values(group_cols).reset_index(drop=True)
        else:
            filtered_run_data = pd.DataFrame()

        num_iterations = len(filtered_run_data)
        data_repeated = pd.concat([data] * num_iterations, ignore_index=True) if num_iterations > 0 else pd.DataFrame()

        return data_repeated, filtered_run_data

    def merge(self,
              include_stats: bool = True,
              include_epoch: bool = False,
              epoch_average: bool = False,
              iteration_mode: Literal['none', 'all', 'max', 'max_limited'] = 'none',
              output_filename: str = "results.csv") -> pd.DataFrame:
        """
        Main merge function with configurable options.

        Args:
            include_stats: Include min/max/avg statistics from requests and vehicles
            include_epoch: Include epoch runtime data
            epoch_average: If True and include_epoch=True, average epoch data; otherwise include all rows
            iteration_mode: How to include iteration data:
                - 'none': No iteration data
                - 'all': All iterations
                - 'max': Maximum iterations only (grouped by subProIter, or by Epoch+subProIter when Epoch exists)
                - 'max_limited': Maximum iterations limited to ≤4 (same grouping)
            output_filename: Name of output CSV file

        Returns:
            Merged DataFrame
        """
        result_frames = []

        for summary_path in tqdm(list(self.root_folder.rglob("summary*.csv")), desc="Processing summaries"):
            # Read base data
            data, parameter_data, suffix = self._read_summary_and_params(summary_path)

            # Add statistics if requested
            if include_stats:
                data = self._add_min_max_stats(data, summary_path, suffix)

            # Handle iteration data
            if iteration_mode != 'none':
                data_repeated, iteration_data = self._add_iteration_data(
                    data, summary_path, suffix, iteration_mode
                )

                # Get epoch data
                if include_epoch:
                    epoch_data = self._add_epoch_data(data, summary_path, suffix, epoch_average)
                    if not epoch_data.empty:
                        num_iterations = len(iteration_data) if not iteration_data.empty else 1
                        epoch_data_repeated = pd.concat([epoch_data] * num_iterations, ignore_index=True)
                    else:
                        epoch_data_repeated = pd.DataFrame()
                else:
                    epoch_data_repeated = pd.DataFrame()

                # Repeat parameters
                num_iterations = len(iteration_data) if not iteration_data.empty else 1
                parameter_data_repeated = pd.concat([parameter_data] * num_iterations, ignore_index=True)

                # Combine all data
                to_concat = [data_repeated, parameter_data_repeated]
                if not epoch_data_repeated.empty:
                    to_concat.append(epoch_data_repeated)
                if not iteration_data.empty:
                    to_concat.append(iteration_data)

                combined = pd.concat(to_concat, axis=1)
            else:
                # No iteration data - simpler merge
                to_concat = [data, parameter_data]

                if include_epoch:
    #                epoch_data = self._add_epoch_data(data, summary_path, suffix, epoch_average)
                    epoch_data = self._add_basic_epoch_data(data, summary_path, suffix, epoch_average)
                    if not epoch_data.empty:
                        to_concat.append(epoch_data)

                combined = pd.concat(to_concat, axis=1)

            # Calculate objective
            if 'Wait_W1' in combined.columns and 'wait/req' in combined.columns:
                combined['AvgObjective'] = (combined['Wait_W1'] * combined['wait/req'] +
                                            combined['Ride_W2'] * combined['tripDelay/req'])

            # Add algorithm name mapping
            required_cols = ['Algorithm', 'Mode', 'vehicleReturn', 'Committed']
            if all(col in combined.columns for col in required_cols):
                combined['Algorithm_Name'] = combined.apply(
                    lambda row: self.algo_map.get(
                        (row['Algorithm'],
                         row['Mode'],
                         str(row['vehicleReturn']).upper(),
                         str(row['Committed']).upper())
                    ), axis=1
                )

            result_frames.append(combined)

        # Merge all results
        if result_frames:
            result_df = pd.concat(result_frames, ignore_index=True)

            # Save to file
            out_path = self.root_folder / output_filename
            result_df.to_csv(out_path, index=False)
            print(f"✓ Merged {len(result_frames)} summaries → {out_path}")

            return result_df
        else:
            print("✗ No summary files found to process")
            return pd.DataFrame()


# Usage examples:
def merge_basic(instance_folder):
    """Basic merge: summary + parameters + stats + objective"""
    merger = ResultMerger(instance_folder)
    return merger.merge(
        include_stats=True,
        include_epoch=False,
        iteration_mode='none',
        output_filename="results_epoch_avg.csv"
    )


def merge_with_averaged_epoch(instance_folder):
    """With averaged epoch data"""
    merger = ResultMerger(instance_folder)
    return merger.merge(
        include_stats=True,
        include_epoch=True,
        epoch_average=True,
        iteration_mode='none',
        output_filename="results_epoch_avg.csv"
    )


def merge_with_all_iterations(instance_folder):
    """With all iteration data"""
    merger = ResultMerger(instance_folder)
    return merger.merge(
        include_stats=True,
        include_epoch=True,
        epoch_average=False,
        iteration_mode='all',
        output_filename="results_full_iterations.csv"
    )


def merge_with_max_iterations_limited(instance_folder):
    """With max iterations (limited to 4)"""
    merger = ResultMerger(instance_folder)
    return merger.merge(
        include_stats=True,
        include_epoch=True,
        epoch_average=False,
        iteration_mode='max_limited',
        output_filename="results_iterations_max4.csv"
    )


def merge_complete(instance_folder):
    """Everything: stats + full epoch data + max iterations"""
    merger = ResultMerger(instance_folder)
    return merger.merge(
        include_stats=False,
        include_epoch=False,
        epoch_average=False,
        iteration_mode='max',
        output_filename="results_complete.csv"
    )

"""
    # Quick use cases:
    my_instance_folder = "Phase_2/" + selected_folder
    merge_basic(my_instance_folder)
    merge_with_averaged_epoch(my_instance_folder)
    merge_complete(my_instance_folder)
    
    # Custom configuration:
    merger = ResultMerger(my_instance_folder)
    df = merger.merge(
        include_stats=True,
        include_epoch=True,
        epoch_average=False,
        iteration_mode='max_limited',
        output_filename="custom_results.csv"
    )
"""