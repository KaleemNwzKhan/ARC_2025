import pandas as pd

# Read the CSV file into a DataFrame
df = pd.read_csv('CUDA_output_file1.csv')

# Print only the 'time' column
print(df['time'].to_string(index=False))
