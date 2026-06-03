import json
import os
import pandas as pd
from typing import List, Dict, Tuple, Optional
import constants as c


def enforce_triangle_inequality(matrix, eps=1e-6):
    n = len(matrix)
    corrected = [row[:] for row in matrix]  # shallow copy per row is fine
    for k in range(n):
        for i in range(n):
            dik = corrected[i][k]
            if dik is None:
                continue
            for j in range(n):
                if i == j:
                    continue
                dkj = corrected[k][j]
                if dkj is None:
                    continue
                cand = dik + dkj
                if corrected[i][j] is None or cand + eps < corrected[i][j]:
                    corrected[i][j] = cand
    return corrected

def fw_using_duration_then_sum_distance(dur_mat, dist_mat, eps=1e-6):
    n = len(dur_mat)
    D = [row[:] for row in dur_mat]
    L = [row[:] for row in dist_mat]
    for k in range(n):
        for i in range(n):
            dik = D[i][k]
            lik = L[i][k]
            if dik is None:
                continue
            for j in range(n):
                if i == j:
                    continue
                dkj = D[k][j]
                lkj = L[k][j]
                if dkj is None:
                    continue
                cand = dik + dkj
                if D[i][j] is None or cand + eps < D[i][j]:
                    D[i][j] = cand
                    # only sum distances if both legs have distances
                    L[i][j] = (None if (lik is None or lkj is None) else lik + lkj)
    return D, L


def json_to_matrix(json_data: Dict) -> Tuple[List[str], List[List[Optional[float]]], List[List[Optional[float]]]]:
    """
    Convert JSON edge list format to matrix format.

    Args:
        json_data: The loaded JSON data

    Returns:
        Tuple of (stop_ids, distance_matrix, duration_matrix)
    """
    # Extract unique stop IDs
    stop_ids = set()
    for pair in json_data['pairs']:
        stop_ids.add(str(pair['origin_id']))
        stop_ids.add(str(pair['destination_id']))

    # Sort numerically if possible, otherwise alphabetically
    try:
        stop_ids = sorted(list(stop_ids), key=lambda x: int(x))
    except ValueError:
        stop_ids = sorted(list(stop_ids))

    n = len(stop_ids)
    stop_id_to_index = {stop_id: idx for idx, stop_id in enumerate(stop_ids)}

    # Initialize matrices with None
    dist_matrix = [[None for _ in range(n)] for _ in range(n)]
    dur_matrix = [[None for _ in range(n)] for _ in range(n)]

    # Set diagonal to 0
    for i in range(n):
        dist_matrix[i][i] = 0
        dur_matrix[i][i] = 0

    # Fill matrices from pairs
    for pair in json_data['pairs']:
        i = stop_id_to_index[str(pair['origin_id'])]
        j = stop_id_to_index[str(pair['destination_id'])]
        dist_matrix[i][j] = pair['distance_m']
        dur_matrix[i][j] = pair['duration_s']

    return stop_ids, dist_matrix, dur_matrix


def matrix_to_json(stop_ids: List[str],
                   dist_matrix: List[List[Optional[float]]],
                   dur_matrix: List[List[Optional[float]]],
                   profile: str = "driving") -> Dict:
    """
    Convert matrices back to JSON edge list format.

    Args:
        stop_ids: List of stop IDs
        dist_matrix: Distance matrix
        dur_matrix: Duration matrix
        profile: Routing profile used

    Returns:
        Dictionary in the JSON format
    """
    n = len(stop_ids)
    pairs = []

    for i in range(n):
        for j in range(n):
            if i != j:  # Skip self-pairs
                pairs.append({
                    "origin_id": stop_ids[i],
                    "destination_id": stop_ids[j],
                    "distance_m": dist_matrix[i][j],
                    "duration_s": dur_matrix[i][j],
                })

    return {
        "profile": profile,
        "count_stops": n,
        "count_pairs": len(pairs),
        "pairs": pairs,
    }


def postprocess_json_matrix(input_json_filename: str,
                            output_json_filename: str = None,
                            output_txt_filename: str = None) -> None:
    """
    Read an existing JSON matrix file, enforce triangle inequality, and save corrected versions.

    Args:
        input_json_filename: Name of the input JSON file (without path)
        output_json_filename: Name for the corrected JSON file. If None, adds '_corrected' suffix
        output_txt_filename: Name for the output TXT file (without extension). If None, derives from output_json_filename
    """
    # Construct input path
    input_path = os.path.join(c.STOP_DIR, input_json_filename)

    # Set output filenames if not provided
    if output_json_filename is None:
        base_name = input_json_filename.replace('.json', '')
        output_json_filename = f"{base_name}.json"

    if output_txt_filename is None:
        output_txt_filename = output_json_filename.replace('.json', '')

    print(f"\n{'=' * 60}")
    print(f"Post-processing: {input_json_filename}")
    print(f"{'=' * 60}")

    # Read the JSON file
    print(f"\n1. Reading {input_json_filename}...")
    try:
        with open(input_path, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File {input_path} not found.")
        return
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in {input_path}.")
        return

    # Convert to matrix format
    print(f"2. Converting to matrix format...")
    stop_ids, dist_matrix, dur_matrix = json_to_matrix(json_data)
    print(f"   Found {len(stop_ids)} stops")

    # Apply triangle inequality enforcement
    print(f"\n4. Enforcing triangle inequality...")
    print("   Processing duration matrix...")
    corrected_dur, corrected_dist = fw_using_duration_then_sum_distance(dur_matrix, dist_matrix)


    # Count changes
    dur_changes = sum(1 for i in range(len(dur_matrix))
                      for j in range(len(dur_matrix))
                      if dur_matrix[i][j] != corrected_dur[i][j] and i != j)

    print(f"   Duration matrix: {dur_changes} entries modified")

    # Convert back to JSON format
    print(f"\n6. Saving corrected JSON file...")
    corrected_json = matrix_to_json(stop_ids, corrected_dist, corrected_dur,
                                    json_data.get('profile', 'driving'))

    output_json_path = os.path.join(c.STOP_DIR, output_json_filename)
    with open(output_json_path, 'w', encoding='utf-8') as f:
        json.dump(corrected_json, f, ensure_ascii=False, indent=2)
    print(f"   Saved to: {output_json_filename}")

    # Convert to TXT format
    print(f"\n7. Converting to TXT format...")
    convert_json_matrix_to_txt(output_json_filename, output_txt_filename)

    print(f"\n{'=' * 60}")
    print(f"Post-processing complete!")
    print(f"Output files:")
    print(f"  - JSON: {output_json_filename}")
    print(f"  - TXT:  {output_txt_filename}.txt")
    print(f"{'=' * 60}\n")


def convert_json_matrix_to_txt(input_json_filename: str, output_txt_filename: str = None) -> None:
    """
    Converts a JSON edge matrix to a TXT file in the required format.

    Args:
        input_json_filename (str): Name of the input JSON file (without path)
        output_txt_filename (str): Name of the output TXT file (without extension).
                                 If None, uses input filename with .txt extension
    """
    # Construct input path
    input_path = os.path.join(c.STOP_DIR, input_json_filename)

    # Set output filename if not provided
    if output_txt_filename is None:
        output_txt_filename = input_json_filename.replace('.json', '')

    # Read the JSON file
    print(f"Reading {input_json_filename}...")
    try:
        with open(input_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File {input_path} not found.")
        return
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in {input_path}.")
        return

    # Extract stop IDs to create a mapping
    stop_ids = set()
    for pair in data['pairs']:
        stop_ids.add(int(pair['origin_id']))
        stop_ids.add(int(pair['destination_id']))

    # Sort numerically
    stop_ids = sorted(list(stop_ids))
    stop_id_to_index = {stop_id: idx for idx, stop_id in enumerate(stop_ids)}

    print(f"Found {len(stop_ids)} unique stops")

    # Create a dictionary for quick lookup of existing pairs
    pair_lookup = {}
    for pair in data['pairs']:
        origin_idx = stop_id_to_index[int(pair['origin_id'])]
        dest_idx = stop_id_to_index[int(pair['destination_id'])]
        pair_lookup[(origin_idx, dest_idx)] = {
            'duration': pair['duration_s'],
            'distance': pair['distance_m']
        }
    # Build the complete matrix data including self-pairs
    matrix_data = []
    for i in range(len(stop_ids)):
        for j in range(len(stop_ids)):
            if i == j:
                # Self-pairs: zero duration and distance
 #               matrix_data.append([i, j, 0, 0])
                matrix_data.append([i, j, 0])
            else:
                # Look up existing pair data
                pair_data = pair_lookup.get((i, j))
                if pair_data:
                    # Round duration and distance to integers
                    duration = round(pair_data['duration']) if pair_data['duration'] is not None else 0
 #                   distance = round(pair_data['distance']) if pair_data['distance'] is not None else 0
 #                   matrix_data.append([i, j, duration, distance])
                    matrix_data.append([i, j, duration])
                else:
                    # Missing pair (shouldn't happen in a complete matrix, but handle gracefully)
 #                   matrix_data.append([i, j, 0, 0])
                    matrix_data.append([i, j, 0])

    # Create DataFrame
 #   df_matrix = pd.DataFrame(matrix_data, columns=['startID', 'EndID', 'duration', 'distance'])
    df_matrix = pd.DataFrame(matrix_data, columns=['startID', 'EndID', 'duration'])

    # Construct output path
    output_path = os.path.join(c.STOP_DIR, output_txt_filename + ".txt")

    # Write to TXT file in the required format
    print(f"Writing to {output_txt_filename}.txt...")
    try:
        with open(output_path, "w", encoding='utf-8') as file:
            # Write header
            file.write("nbLocations = ")
            file.write(str(len(stop_ids)))
            file.write("\n\n")
            file.write("COLUMNS\n\n")

            # Write column names
            for col in df_matrix.columns:
                file.write(col + "\n")

            # Write data section header
            file.write("\nDURATION_INFO\n")

            # Write the data (without pandas headers or index)
            df_as_string = df_matrix.to_string(header=False, index=False)
            file.write(df_as_string)

        print(f"Successfully created {output_txt_filename}.txt")
        print(f"Total pairs written: {len(matrix_data)}")

        # Print summary statistics
        null_durations = sum(1 for _, _, dur in matrix_data if dur is None or dur == 0)
 #       null_distances = sum(1 for _, _, _, dist in matrix_data if dist is None or dist == 0)

        print(f"Pairs with zero/null duration: {null_durations}")
 #       print(f"Pairs with zero/null distance: {null_distances}")

    except Exception as e:
        print(f"Error writing output file: {e}")


"""
Example Usage:
-------------
    postprocess_json_matrix("edge_matrix.json")
    convert_json_matrix_to_txt("riley_edge_matrix.json", "riley_edge_time_matrix")
    convert_json_matrix_to_txt("edge_matrix.json", "edge_time_matrix")

"""