import json
import random
import os


# Define the structure of each resource as a dictionary
def create_resource(id):
    return {
        "ID": id,
        "Inventory": random.randint(1, 39),  # Random value smaller than 40
        "Capacity": random.randint(10, 40),  # Random value between 10 and 40
        "Location": 666,  # Fixed location
        "StartTime": 25200,  # Fixed start time
        "EndTime": 90000,  # Fixed end time
        "ResourceType": "type1",  # Fixed resource type
        "EndShiftInventory": random.randint(10, 40),  # Random value between 10 and 40
        "numTasks": 0  # Fixed number of tasks
    }

# Generate a list of 10 resources
resources = [create_resource(id) for id in range(1, 11)]

# Define the file path
# Display the current working directory for debugging purposes
print("Current Working Directory:", os.getcwd())

# Define the folder path relative to the current working directory
folder_path = "../../data"
# Ensure the directory exists
os.makedirs(folder_path, exist_ok=True)

# Join the folder path and the file name to create a complete file path
file_path = os.path.join(folder_path, 'resources.json')

# Assuming 'resources' list is already defined and populated as per previous code
# Write the list of resources to a JSON file
with open(file_path, 'w') as json_file:
    json.dump(resources, json_file, indent=4)

print(f"JSON file '{file_path}' has been created with 10 resources.")