import os
import pandas as pd

# Specify the directory containing the CSV files
folder_path = "/dataset/Communication/Producer_BlindSpot_Latencies"  # Replace with the actual folder path

# Output file
output_file = os.path.join(folder_path, "/workspace/Results/Latency_Results/Publisher_Latencies.txt")

# Initialize a list to store all data
combined_data = []

# Loop through all CSV files from 0.csv to 39.csv
for i in range(40):  # Files range from 0.csv to 39.csv
    file_name = f"{i}.csv"
    file_path = os.path.join(folder_path, file_name)

    if os.path.exists(file_path):
        # Read the CSV file, skipping only the first row (header)
        df = pd.read_csv(file_path, skiprows=1, header=None)  # Skip the first row only
        combined_data.append(df.to_csv(index=False, header=False, sep=" ", lineterminator="\n").strip())  
    else:
        print(f"Warning: {file_name} does not exist in {folder_path}")

# Write the concatenated data to a single text file
with open(output_file, "w") as outfile:
    outfile.write("\n".join(combined_data) + "\n")

print(f"Combined data saved to {output_file}")

