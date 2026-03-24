import numpy as np
import matplotlib.pyplot as plt

# Initialize lists to hold data from files
RTE_X, RTE_Y = [], []
RRE_X, RRE_Y = [], []
Leader_RRE_Y, Direct_RRE_Y = [], []

# Read data from RTE_V2V_Direct.txt
with open("RTE_V2V_Direct.txt", "r") as f:
    for line in f:
        line = line.split(' ')
        RTE_X.append(float(line[0].strip()))
        RTE_Y.append(float(line[-1].strip()) * 100)  # Convert to cm

# Read data from RTE_V2V_Lead.txt
with open("RTE_V2V_Lead.txt", "r") as g:
    for line in g:
        line = line.split(' ')
        RRE_X.append(float(line[0].strip()))
        RRE_Y.append(float(line[-1].strip()) * 100)  # Convert to cm

# Read data from Direct_RRE.txt
with open("RRE_V2V_Direct.txt", "r") as i:
    for line in i:
        line = line.split(' ')
        Direct_RRE_Y.append(float(line[-1].strip()))  # Convert to cm

# Read data from Leader_RRE.txt
with open("RRE_V2V_Lead.txt", "r") as h:
    for line in h:
        line = line.split(' ')
        Leader_RRE_Y.append(float(line[-1].strip()))  # Convert to cm

# Apply filtering based on RRE_Y threshold and keep corresponding values for all datasets
filtered_RTE_Y, filtered_RRE_Y, filtered_Leader_RRE_Y, filtered_Direct_RRE_Y = [], [], [], []

for rre_y, rte_y, leader_rre, direct_rre in zip(RRE_Y, RTE_Y, Leader_RRE_Y, Direct_RRE_Y):
    if rre_y <= 10 and leader_rre <= 0.1 :  # Filtering condition
        filtered_RRE_Y.append(rre_y)
        filtered_RTE_Y.append(rte_y)
        filtered_Leader_RRE_Y.append(leader_rre)
        filtered_Direct_RRE_Y.append(direct_rre)

# Define frames for plotting
Frames = np.arange(0, len(filtered_RTE_Y), 1).tolist()

# Compute statistics
def compute_stats(data):
    return np.mean(data), np.percentile(data, 95), np.percentile(data, 99)

RTE_mean, RTE_95th, RTE_99th = compute_stats(filtered_RTE_Y)
RRE_mean, RRE_95th, RRE_99th = compute_stats(filtered_RRE_Y)
Leader_RRE_mean, Leader_RRE_95th, Leader_RRE_99th = compute_stats(filtered_Leader_RRE_Y)
Direct_RRE_mean, Direct_RRE_95th, Direct_RRE_99th = compute_stats(filtered_Direct_RRE_Y)

# 📌 **Plot 1: Map Alignment vs Leader Alignment**
plt.figure(figsize=(12, 6))
plt.plot(Frames, filtered_RTE_Y, color='red', linewidth=1, label=f'Alignment through Map (Mean: {RTE_mean:.2f} cm)')
plt.plot(Frames, filtered_RRE_Y, color='green', linewidth=1, label=f'Our Approach (Mean: {RRE_mean:.2f} cm)')

# Mean lines
plt.axhline(y=RTE_mean, color='red', linestyle='--', linewidth=2)
plt.axhline(y=RRE_mean, color='green', linestyle='--', linewidth=2)

# Labels and title
plt.xlabel('Frames', fontsize=18)
plt.ylabel('RTE (cm)', fontsize=18)
plt.legend(fontsize=18, loc='upper right')
plt.grid(alpha=0.3)

# # Save and show plot
plt.tight_layout()
plt.savefig('Map_vs_Leader.png')
plt.show()

# 📌 **Plot 2: Leader_RRE vs Direct_RRE**
plt.figure(figsize=(12, 6))
plt.plot(Frames, filtered_Direct_RRE_Y, color='red', linewidth=1, label=f'Alignment through Map(Mean: {Direct_RRE_mean:.2f} degree)')
plt.plot(Frames, filtered_Leader_RRE_Y, color='green', linewidth=1, label=f'Our Approach (Mean: {Leader_RRE_mean:.2f} degree)')


# Mean lines
plt.axhline(y=Direct_RRE_mean, color='red', linestyle='--', linewidth=2)
plt.axhline(y=Leader_RRE_mean, color='green', linestyle='--', linewidth=2)

# Labels and title
plt.xlabel('Frames', fontsize=18)
plt.ylabel('RRE (degree)', fontsize=18)
plt.legend(fontsize=18, loc='upper right')
plt.grid(alpha=0.3)

# Save and show plot
plt.tight_layout()
plt.savefig('LeaderRRE_vs_DirectRRE.pdf')
plt.show()

# 📌 **Compute CDFs**
def compute_cdf(data):
    sorted_data = np.sort(data)
    cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)
    return sorted_data, cdf

RTE_sorted, RTE_cdf = compute_cdf(filtered_RTE_Y)
RRE_sorted, RRE_cdf = compute_cdf(filtered_RRE_Y)
Leader_RRE_sorted, Leader_RRE_cdf = compute_cdf(filtered_Leader_RRE_Y)
Direct_RRE_sorted, Direct_RRE_cdf = compute_cdf(filtered_Direct_RRE_Y)

# 📌 **Plot 3: CDF for Map Alignment vs Leader Alignment**
plt.figure(figsize=(12, 6))
plt.plot(RTE_sorted, RTE_cdf, color='red', linewidth=4, label="Map-based Alignment")
plt.plot(RRE_sorted, RRE_cdf, color='green', linewidth=4, label="Our Approach")

# Labels and title
plt.xlabel("Alignment Error (cm)", fontsize=25)
plt.ylabel("CDF", fontsize=25)
plt.legend(fontsize=18, loc='lower right')
plt.grid(alpha=0.3)
plt.tick_params(axis='both', which='major', labelsize=20)  # Major ticks

# Save and show plot
plt.tight_layout()
plt.savefig('CDF_Map_vs_Leader.png')
plt.show()

# 📌 **Plot 4: CDF for Leader_RRE vs Direct_RRE**
plt.figure(figsize=(12, 6))
plt.plot(Direct_RRE_sorted, Direct_RRE_cdf, color='red', linewidth=2, label="Alignment through Map")
plt.plot(Leader_RRE_sorted, Leader_RRE_cdf, color='green', linewidth=2, label="Our Approach")


# Labels and title
plt.xlabel("RRE (degree)", fontsize=18)
plt.ylabel("CDF", fontsize=18)
plt.legend(fontsize=18, loc='lower right')
plt.grid(alpha=0.3)

# # Save and show plot
plt.tight_layout()
plt.savefig('CDF_LeaderRRE_vs_DirectRRE.pdf')
plt.show()
