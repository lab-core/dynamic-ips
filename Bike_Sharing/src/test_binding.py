import sys
import json
import os

# Import the CG_binding module
import CG_binding

# Create an instance of the Solver class
solver_instance = CG_binding.Solver()

# Paths to the input JSON files
solver_parameters_path = "datasets/Test1/solver_parameters.json"
duration_path = "datasets/Test1/edge_time_matrix.json"
task_path = "datasets/Test1/tasks_by_location.json"
resource_path = "datasets/data/Test1/vehicles.json"

# Load data from JSON files
with open(solver_parameters_path, 'r') as file:
    parameters_data = json.load(file)

with open(duration_path, 'r') as file:
    duration_data = json.load(file)

with open(task_path, 'r') as file:
    task_data = json.load(file)

with open(resource_path, 'r') as file:
    vehicles_data = json.load(file)

# Convert JSON data to strings
parameters_str = json.dumps(parameters_data)
duration_str = json.dumps(duration_data)
tasks_str = json.dumps(task_data)
vehicles_str = json.dumps(vehicles_data)

# Create a Solver instance and call the solve method
solver_instance.createInstance(duration_str, parameters_str, tasks_str, vehicles_str)
result = solver_instance.solveCG()

print("Solver result:", result)
