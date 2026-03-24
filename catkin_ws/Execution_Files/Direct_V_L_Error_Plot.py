import numpy as np
import matplotlib.pyplot as plt

# Load the data, assuming space or tab as a delimiter
file_path_direct = "RTE_Direct.txt"  # Path for Direct alignment
file_path_ndt = "RTE_NDT.txt"  # Path for NDT alignment

data_direct = np.loadtxt(file_path_direct)
data_ndt = np.loadtxt(file_path_ndt)

# Extract the second column and scale by 100
rte_values_direct = data_direct[:, 1] * 100
rte_values_ndt = data_ndt[:, 1] * 100

# Determine the maximum length
max_length = max(len(rte_values_direct), len(rte_values_ndt))

# Pad shorter array with zeros
rte_values_direct = np.pad(rte_values_direct, (0, max_length - len(rte_values_direct)), 'constant')
rte_values_ndt = np.pad(rte_values_ndt, (0, max_length - len(rte_values_ndt)), 'constant')

# Generate frame numbers (x-axis)
frame_numbers = np.arange(max_length)

# Compute means
mean_rte_direct = np.mean(rte_values_direct)
mean_rte_ndt = np.mean(rte_values_ndt)

# Plot
plt.figure(figsize=(10, 5))
plt.plot(frame_numbers, rte_values_direct, label="RTE Direct (cm)", color="b", linewidth=3)
plt.plot(frame_numbers, rte_values_ndt, label="RTE NDT (cm)", color="g", linewidth=3)

# Mean lines
plt.axhline(mean_rte_direct, color="b", linestyle="--", linewidth=4, label=f"Mean Direct: {mean_rte_direct:.2f} cm")
plt.axhline(mean_rte_ndt, color="g", linestyle="--", linewidth=4, label=f"Mean NDT: {mean_rte_ndt:.2f} cm")

# Set x-axis interval to 1000
plt.xticks(np.arange(0, max_length, 1000))

# Labels and title
plt.xlabel("Frame Number")
plt.ylabel("RTE (cm)")
plt.title("RTE Comparison: Direct vs NDT")
plt.legend()
plt.grid(True)

# Show plot
plt.show()


