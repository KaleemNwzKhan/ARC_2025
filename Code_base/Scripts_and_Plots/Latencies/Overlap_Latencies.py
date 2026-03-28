import os

# Specify the directory containing the Overlap files
folder_path = "/dataset/Leader_Vehicle_Overlap/Overlap_Latencies"  # Replace with the actual folder path

# Initialize a list to store all data
combined_data = []

# Loop through all Overlap_*.txt files
for i in range(40):  # Assuming files range from Overlap_0.txt to Overlap_39.txt
    file_name = f"Overlap_{i}.txt"
    file_path = os.path.join(folder_path, file_name)
    
    if os.path.exists(file_path):
        # Read the file and skip the first line
        with open(file_path, "r") as file:
            lines = file.readlines()
            if len(lines) > 1:  # Ensure there are numbers after the title
                combined_data.extend(lines[1:])  # Skip the title (first line)
    else:
        print(f"Warning: {file_name} does not exist in {folder_path}")

# Write the concatenated data to a single file
output_file = os.path.join(folder_path, "/workspace/Results/Latency_Results/Overlap_Estimation.txt")
with open(output_file, "w") as outfile:
    outfile.writelines(combined_data)

print(f"Combined data saved to {output_file}")
