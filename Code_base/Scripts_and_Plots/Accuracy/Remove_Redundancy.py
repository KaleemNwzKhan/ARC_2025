import os

# Possible directories containing the results
possible_dirs = [
    "/workspace/Results/Alignment_Results",
    "/home/kk5271/RIT_Research/ARC_Repo/Code_base/Results/Alignment_Results"
]

base_dir = None
for d in possible_dirs:
    if os.path.exists(d):
        base_dir = d
        break

if not base_dir:
    print("Error: Could not find Alignment_Results directory.")
    exit(1)

print(f"Using directory: {base_dir}")

files_to_process = [
    "RRE_V2V_Direct.txt",
    "RRE_V2V_Lead.txt",
    "RTE_V2V_Direct.txt",
    "RTE_V2V_Lead.txt"
]

# Parameters
num_files = 40  # Total files
values_per_file = 150  # Values per file
remove_step = 150  # Number of values to discard in each cycle

for filename in files_to_process:
    file_path = os.path.join(base_dir, filename)
    print(f"Processing {filename}...")
    
    # Read all values into a list (keeping row number)
    data = []
    if not os.path.exists(file_path):
        print(f"Warning: {file_path} not found. Skipping.")
        continue

    with open(file_path, "r") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 2:
                continue  # Skip malformed lines
            try:
                row_num = int(parts[0])  # Extract row number
                value = float(parts[1])  # Extract the second column
                data.append((row_num, value))
            except (ValueError, IndexError):
                continue

    # Start cycle from 39 files
    remaining_files = num_files - 1  
    row_counter = 1  # Sequential row numbering
    cleaned_values = []

    # Local copy of data for processing
    temp_data = list(data)

    while remaining_files > 0:
        keep_size = remaining_files * values_per_file  # Values to keep in this cycle
        extracted_values = temp_data[:keep_size]  # Take first `keep_size` values
        
        # Store extracted values with new row numbering
        for row in extracted_values:
            cleaned_values.append(f"{row_counter} {row[1]:.8f}")  # Format for precision
            row_counter += 1
        
        # Remove the processed and discarded values
        temp_data = temp_data[keep_size + remove_step:]  
        
        # Reduce remaining files count
        remaining_files -= 2  

    if not cleaned_values:
        print(f"Warning: No cleaned values generated for {filename}. Skipping write.")
        continue

    # Write the cleaned values back to the original file
    with open(file_path, "w") as f:
        for line in cleaned_values:
            f.write(f"{line}\n")

    print(f"Successfully cleaned {filename}. Original size: {len(data)}, Cleaned size: {len(cleaned_values)}")

print("All files processed.")
