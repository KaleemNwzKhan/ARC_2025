import os

# Specify the main directory containing all vehicle directories
main_directory = "/dataset/Leader_Vehicle_Overlap"

# Initialize a list to store all latencies
all_latencies = []

# Loop through all vehicle directories
for i in range(40):
    # Construct the directory and file path
    directory = os.path.join(main_directory, f"Vehicle_{i}")
    file_path = os.path.join(directory, "Latencies.txt")
    
    # Read the file and append its data to all_latencies
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            latencies = file.readlines()
            all_latencies.extend(latencies)
    else:
        print(f"Warning: {file_path} does not exist.")

# Write the concatenated latencies to a single file
output_file = os.path.join(main_directory, "/workspace/Results/Latency_Results/Anchor_Alignment.txt")
with open(output_file, "w") as outfile:
    outfile.writelines(all_latencies)

print(f"Concatenated latencies saved to {output_file}")
