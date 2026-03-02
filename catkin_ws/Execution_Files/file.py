import os

# Directory path where folders V_0, V_1, ..., V_43 are located
base_dir = "PCDs"

# List all folders in the base directory
folders = [f for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))]

# Sort the folders based on their names
folders.sort()

# Keep track of the expected folder number
expected_number = 0

# Loop through the sorted folders
for folder in folders:
    # Extract the number from the folder name (e.g., V_0 -> 0)
    folder_number = int(folder.split('_')[1])

    # If the folder number does not match the expected number, rename it
    if folder_number != expected_number:
        new_name = f"V_{expected_number}"
        old_path = os.path.join(base_dir, folder)
        new_path = os.path.join(base_dir, new_name)

        # Rename the folder
        print(f"Renaming {folder} to {new_name}")
        os.rename(old_path, new_path)

    # Increment the expected number for the next folder
    expected_number += 1

# If there are still missing folders after the loop, create the missing ones
for i in range(expected_number, 44):
    missing_folder = f"V_{i}"
    missing_path = os.path.join(base_dir, missing_folder)

    # Create the missing folder
    if not os.path.exists(missing_path):
        print(f"Creating missing folder {missing_folder}")
        os.makedirs(missing_path)
