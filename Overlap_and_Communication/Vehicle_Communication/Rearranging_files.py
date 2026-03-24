import os

# Input and output base directories
input_base_dir = "Required"  # Replace with actual input directory
output_base_dir = "output_directory"  # Replace with actual output directory

# Ensure output base directory exists
os.makedirs(output_base_dir, exist_ok=True)

# Get list of vehicle directories
vehicle_dirs = sorted([d for d in os.listdir(input_base_dir) if d.startswith("Vehicle_")])

# Number of frames (assumed based on file names)
num_frames = 150  # Adjust if needed

# Process each frame file-wise
for frame_num in range(1, num_frames + 1):
    frame_data = {}  # Stores required numbers for each vehicle

    # Read all vehicles' frame files
    for vehicle in vehicle_dirs:
        file_path = os.path.join(input_base_dir, vehicle, f"{frame_num}.txt")
        if os.path.exists(file_path):
            with open(file_path, "r") as f:
                for line in f:
                    value, target_vehicle = line.strip().split()
                    if target_vehicle not in frame_data:
                        frame_data[target_vehicle] = set()  # Use a set to avoid duplicates
                    frame_data[target_vehicle].add(value)  # Add to set (ensures unique values)

    # Write the processed data to new vehicle directories
    for vehicle, values in frame_data.items():
        vehicle_output_dir = os.path.join(output_base_dir, vehicle)
        os.makedirs(vehicle_output_dir, exist_ok=True)
        
        output_file_path = os.path.join(vehicle_output_dir, f"{frame_num}.txt")
        with open(output_file_path, "w") as f:
            f.write("\n".join(sorted(values)) + "\n")  # Sort values for consistency

print("Processing complete. Check the output directory.")

