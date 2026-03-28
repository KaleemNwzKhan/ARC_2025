import os

# Input directory containing the 39 txt files
input_dir = "Producer_Assignment_Latencies"  # Replace with actual directory path

# Output file
output_file = "Producer_Assignment_Latencies.txt"

# List of files in order from 0.txt to 38.txt
file_list = [os.path.join(input_dir, f"{i}.txt") for i in range(40)]

# Open output file for writing
with open(output_file, "w") as outfile:
    for file_path in file_list:
        if os.path.exists(file_path):
            with open(file_path, "r") as infile:
                outfile.writelines(infile.readlines())  # Append all lines

print(f"Merged {len(file_list)} files into {output_file}")

