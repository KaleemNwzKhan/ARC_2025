import os

# Define the input folder and output file
input_folder = "/dataset/Communication/BlindSpots_Size"  # Replace with the actual directory
output_file = "/workspace/Results/Network_Results/Publisher_Sizes.txt"

# Number of files and entries per file
num_files = 39  
num_entries = 150  

# Initialize a list to store the sum of values
sums = [0.0] * num_entries  # Use float for decimal numbers

# Read and sum values from each file
for i in range(num_files):
    file_path = os.path.join(input_folder, f"{i}.txt")  

    if os.path.exists(file_path):
        with open(file_path, "r") as f:
            lines = f.readlines()
            for j in range(min(len(lines), num_entries)):  # Ensure we don't exceed 150 entries
                sums[j] += float(lines[j].strip())  # Convert to float and sum

# Write the summed values to the output file
with open(output_file, "w") as out_file:
    out_file.write("\n".join(map(str, sums)) + "\n")

print(f"Summed data saved to {output_file}")

