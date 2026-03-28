import os

# Specify the main directory containing all vehicle directories
main_directory = "/dataset/Vehicles"

# Initialize a list to store all processed latencies
all_latencies = []

# Loop through all vehicle directories
for i in range(40):
    # Construct the directory and file path
    directory = os.path.join(main_directory, f"Vehicle_{i}")
    file_path = os.path.join(directory, "Latencies.txt")
    
    # Read the file and process its data
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            latencies = [float(line.strip()) for line in file.readlines()]  # Convert to float
            
            if len(latencies) > 1:
                # Compute the mean of all entries except the first
                mean_value = sum(latencies[1:]) / (len(latencies) - 1)
                latencies[0] = mean_value  # Replace the first entry with the mean
            
            # Append the processed latencies as strings with newline characters
            all_latencies.extend(f"{latency}\n" for latency in latencies)
    else:
        print(f"Warning: {file_path} does not exist.")

# Write the concatenated latencies to a single file
output_file = os.path.join(main_directory, "/workspace/Results/Latency_Results/Localization.txt")
with open(output_file, "w") as outfile:
    outfile.writelines(all_latencies)

print(f"Processed and concatenated latencies saved to {output_file}")
