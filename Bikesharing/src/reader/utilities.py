import pandas as pd
import os
import glob


def convert_csv_folder_to_json(folder_path):
    """
    Convert all CSV files in the specified folder to JSON format.

    Parameters:
    - folder_path: The path to the folder containing the CSV files.

    This function will save each converted file as a JSON with the same base name
    as the CSV file but with a .json extension, in the same folder.
    """
    # Get the parent directory of the folder_path
    parent_directory = os.path.dirname(folder_path)

    # Create the 'tasks_data' folder within the parent directory
    tasks_data_folder = os.path.join(parent_directory, 'tasks_data')

    # Ensure the 'tasks_data' folder exists, create it if it doesn't
    os.makedirs(tasks_data_folder, exist_ok=True)

    # Use glob to find all files in the folder (regardless of extension)
    # Modify the pattern as needed to match your files
    files = glob.glob(os.path.join(folder_path, '*'))

    # Dictionary to keep track of converted files for reference
    converted_files = {}

    for file_path in files:
        # Check if the file is a directory to skip directories
        if os.path.isdir(file_path):
            continue

        # Read the file into a DataFrame assuming it's structured as a CSV
        df = pd.read_csv(file_path)

        # Convert the DataFrame to JSON
        json_data = df.to_json(orient='records', lines=False)

        # Construct the JSON file name by adding .json extension
        json_file_name = os.path.splitext(os.path.basename(file_path))[0] + '.json'
        json_file_path = os.path.join(tasks_data_folder, json_file_name)

        # Write the JSON data to a new file in the 'tasks_data' folder
        with open(json_file_path, 'w') as json_file:
            json_file.write(json_data)

        # Add the path to the converted file to the dictionary
        converted_files[file_path] = json_file_path

    return converted_files


# Specify the path to the folder containing your CSV files
print("Current Working Directory:", os.getcwd())
folder_path = "../../data/costs"

# Convert the CSV files in the folder to JSON format
converted_files = convert_csv_folder_to_json(folder_path)

# Print the paths of the converted JSON files for reference
for csv, json in converted_files.items():
    print(f"Converted {csv} to {json}")
