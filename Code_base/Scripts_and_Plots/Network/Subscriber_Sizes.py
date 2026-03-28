import os

# Define the three main folders
folders = ["/dataset/Communication/BlindSpots", "/dataset/Communication/Required", "/dataset/Communication/Vehicles_Cells"]  # Replace with actual folder names

# Number of vehicles and frames
num_vehicles = 40  # Vehicle_0 to Vehicle_39
num_frames = 150  # 1.txt to 150.txt

# Output file
output_file = "/workspace/Results/Network_Results/Subscriber_Sizes.txt"

# List to store summed sizes for each frame
frame_sizes = []

# Iterate through each frame
for frame_num in range(1, num_frames + 1):
    total_size = 0  # Sum of file sizes for this frame across all folders

    # Iterate through each vehicle
    for vehicle_id in range(num_vehicles):
        vehicle_folder = f"Vehicle_{vehicle_id}"
        file_name = f"{frame_num}.txt"

        # Sum sizes from all three folders
        for folder in folders:
            file_path = os.path.join(folder, vehicle_folder, file_name)
            if os.path.exists(file_path):
                with open(file_path, "rb") as f:
                    total_size += len(f.read())  # Get content size in bytes

    # Convert bytes to KB (integer division)
    frame_sizes.append(total_size // 1024)

# Save the result
with open(output_file, "w") as out_file:
    out_file.write("\n".join(map(str, frame_sizes)) + "\n")

print(f"Size data saved to {output_file}")

