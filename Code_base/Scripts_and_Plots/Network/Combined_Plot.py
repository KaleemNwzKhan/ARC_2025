import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
from matplotlib.ticker import MaxNLocator, FormatStrFormatter

# ------------------- 1. Data Combination & Conversion -------------------
def combine_and_convert_sizes():
    input_files = ["/workspace/Results/Network_Results/Publisher_Sizes.txt", "/workspace/Results/Network_Results/Subscriber_Sizes.txt", "/workspace/Results/Network_Results/Meta_Data_Sharing.txt", "/workspace/Results/Network_Results/point_cloud_sizes.txt"]
    output_file_kb = "RSU_E2E_Bandwidth.txt"
    output_file_mb = "ASF_Vehicle.txt"
    
    num_entries = 150  
    sums = [0.0] * num_entries 
    
    # Read and sum KBs
    for file_path in input_files:
        if os.path.exists(file_path):
            with open(file_path, "r") as f:
                lines = f.readlines()
                for i in range(min(len(lines), num_entries)):
                    sums[i] += float(lines[i].strip())
        else:
            print(f"Warning: {file_path} does not exist.")
            
    # Write summed KB values
    with open(output_file_kb, "w") as out_file:
        out_file.write("\n".join(map(str, sums)) + "\n")
    print(f"Summed data (KB) saved to {output_file_kb}")
    
    # Convert to MB and write
    with open(output_file_mb, "w") as outfile:
        for size_kb in sums:
            size_mb = size_kb / 1024.0  # Convert KB to MB
            outfile.write(f"{size_mb}\n")
    print(f"Converted sizes (MB) saved to {output_file_mb}")
    
    return output_file_mb

# ------------------- 2. Font & Style Configuration -------------------
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['font.size'] = 20
mpl.rcParams['axes.titlesize'] = 25
mpl.rcParams['axes.labelsize'] = 25
mpl.rcParams['xtick.labelsize'] = 20
mpl.rcParams['ytick.labelsize'] = 20
mpl.rcParams['legend.fontsize'] = 25
mpl.rcParams['figure.titlesize'] = 25

# ------------------- 3. Plotting Logic -------------------
def read_bandwidth_values(file_path):
    """Reads bandwidth values from a file and returns them as a list (in MBs)."""
    values = []
    
    # Try looking in the AutoCast directory if it's not in the root directory
    if not os.path.exists(file_path) and os.path.exists(os.path.join("AutoCast", file_path)):
        file_path = os.path.join("AutoCast", file_path)

    if not os.path.exists(file_path):
        print(f"Error: Could not find {file_path} to read.")
        return values
        
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            parts = line.strip().split()
            if parts:
                try:
                    value = float(parts[0])  # Assume first value in each line is bandwidth
                    values.append(value)
                except ValueError:
                    continue
    return values

def convert_to_mbps_repeat(values, frames_per_second=10):
    """Keeps first 7.5s of data and repeats it in the same order to total 15s."""
    frames_7_5s = int(frames_per_second * 7.5)  # 75 frames

    if len(values) < frames_7_5s:
        print(f"Warning: Not enough data for 7.5 seconds (found {len(values)}). Padding with zeros.")
        values.extend([0.0] * (frames_7_5s - len(values)))

    # Keep first 7.5 seconds
    original = values[:frames_7_5s]

    # Repeat the same segment again
    repeated = original + original  # Now 150 frames total (15s)

    # Convert to per-second Mbps
    mbps_values = []
    for i in range(0, len(repeated), frames_per_second):
        mb_sum = sum(repeated[i:i + frames_per_second])
        mbps = mb_sum * 8  # MB to Mbps
        mbps_values.append(mbps)

    return mbps_values

def plot_multiple_bandwidth(files, labels, colors, output_pdf="bandwidth_comparison_combined.pdf"):
    """Plots per-second bandwidth (in Mbps) values with compact figure and styling."""
    fig, ax = plt.subplots(figsize=(18, 6))  # Compact height

    for file_path, label, color in zip(files, labels, colors):
        values = read_bandwidth_values(file_path)
        if not values:
            print(f"No valid values found in {file_path}")
            continue

        mbps_values = convert_to_mbps_repeat(values)
        seconds = list(range(1, len(mbps_values) + 1))
        mean_value = np.ceil(np.mean(mbps_values))

        ax.plot(seconds, mbps_values, marker='o', linestyle='-', linewidth=10, markersize=15,
                label=f'{label} [avg: {mean_value:.0f} Mbps]', color=color)

    ax.set_xlabel("Time (seconds)",fontsize=42)
    ax.set_ylabel("Bandwidth (Mbps)",fontsize=42)
    
    ax.tick_params(axis='both', which='major', labelsize=42)
    ax.set_ylim(0, 100)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.0f'))

    ax.tick_params(axis='both', which='major', length=10, width=2)
    ax.tick_params(axis='both', which='major', labelsize=45)  # increase tick size

    fig.legend(
        loc='upper center',
        ncol=len(labels),  # Dynamic columns based on number of plots
        bbox_to_anchor=(0.47, 1.55),
        bbox_transform=ax.transAxes,
        fontsize=40,
        frameon=False,
        columnspacing=0.5  # smaller value = less space between columns
    )

    plt.tight_layout(pad=1.0)
    plt.savefig(output_pdf, bbox_inches='tight')
    print(f"Plot saved as {output_pdf}")
    # plt.show() # Disable to run quietly via command line

if __name__ == '__main__':
    # 1. Combine and convert files
    combined_mb_file = combine_and_convert_sizes()

    # 2. Setup Plot Files. Keeping AutoCast, and plotting the Combined file as ARC.
    # We use AutoCast color "#00008B" and ARC color "peru" (or "green"). Let's use peru.
    files = [
        combined_mb_file
    ]
    labels = [
        "ARC"
    ]
    colors = [
        "#00008B"     # Color for ARC 
    ]

    # 3. Plot exactly as Plot.py
    plot_multiple_bandwidth(files, labels, colors, output_pdf="/workspace/Scripts_and_Plots/Network/Bandwidth_Plot.pdf")
