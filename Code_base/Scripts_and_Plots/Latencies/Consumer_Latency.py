import pandas as pd
import os

# Input directory containing CSV files
input_directory = "/dataset/Communication/Subscriber_BlindSpot_Latencies"  # Replace with your directory path
output_file = "/workspace/Results/Latency_Results/Subscriber_Latency.txt"  # Final concatenated output file

# Get a sorted list of CSV files in numerical order
csv_files = sorted(
    [os.path.join(input_directory, f) for f in os.listdir(input_directory) if f.endswith(".csv")],
    key=lambda x: int(os.path.splitext(os.path.basename(x))[0])
)

# Initialize an empty list to store the E2E_Latency column
all_results = []

# Process each CSV file
for csv_file in csv_files:
    # Read the CSV file normally to get the column names
    df = pd.read_csv(csv_file)
    
    # Check if the column "E2E_Latency" exists in the CSV file
    if "E2E_Latency" in df.columns:
        # Extract the E2E_Latency column, skipping the first row of data (which should be index 0)
        e2e_latency = df["E2E_Latency"]  # Skips the first row
        all_results.append(e2e_latency)
        print(f"Processed {csv_file}")
    else:
        print(f"Skipping {csv_file} as it does not contain 'E2E_Latency' column.")

# Concatenate all results into a single series
final_result = pd.concat(all_results, ignore_index=True)

# Save the final result to a single TXT file
final_result.to_csv(output_file, index=False, header=False)

print(f"All E2E_Latency values concatenated and saved to {output_file}")
