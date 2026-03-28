import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams, ticker
import matplotlib as mpl
import os

# ------------------- Global Plot Style -------------------
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

# ------------------- Data Processing -------------------
def calculate_e2e_latency():
    # List of file names
    file_names = [
        "/workspace/Results/Latency_Results/Localization.txt",
        "/workspace/Results/Latency_Results/Overlap_Estimation.txt",
        "/workspace/Results/Latency_Results/Anchor_Alignment.txt",
        "/workspace/Results/Latency_Results/Subscriber_Latency.txt",
        "/workspace/Results/Latency_Results/Publisher_Latencies.txt",
        "/workspace/Results/Latency_Results/Publisher_Assignment_Latencies.txt"
    ]

    # Total number of expected entries in each file
    expected_entries = 6000

    # Initialize a dictionary to store data from each file
    data = {}

    # Read data from each file
    for file_name in file_names:
        if not os.path.exists(file_name):
            print(f"Warning: {file_name} not found.")
            continue
            
        with open(file_name, "r") as file:
            values = np.array([float(line.strip()) for line in file.readlines()])
            
            # Check for missing values
            if len(values) < expected_entries:
                mean_value = np.mean(values) if len(values) > 0 else 0
                missing_count = expected_entries - len(values)
                values = np.append(values, [mean_value] * missing_count)
                print(f"{file_name}: {missing_count} entries were missing. Appended mean value {mean_value:.2f}.")
            
            data[file_name] = values

    if not data:
        print("No data loaded. Cannot calculate E2E Latency.")
        return np.array([])

    # Calculate E2E_Latency as the sum of all component latencies
    e2e_latency = sum(data.values())
    
    # Save to file for record keeping
    np.savetxt("ASF.txt", e2e_latency, fmt="%.6f")
    print("E2E Latency values saved to ASF.txt")
    
    return e2e_latency

# ------------------- Plotting Function -------------------
def plot_arc_latency(arc_latencies):
    if len(arc_latencies) == 0:
        print("No latencies to plot.")
        return

    frames = np.arange(1, len(arc_latencies) + 1)

    # -------- Downsampling factor --------
    step = 20

    frames_ds = frames[::step]
    arc_ds = arc_latencies[::step]

    # Mean values
    mean_arc = np.mean(arc_latencies)

    # Plot
    fig, ax = plt.subplots(figsize=(17, 4))

    ax.plot(
        frames_ds,
        arc_ds,
        color='peru',
        marker='o',
        linewidth=6,
        markersize=10,
        label=f"ARC (avg: {mean_arc:.1f} ms)"
    )

    ax.set_xlabel("Frame number", fontsize=40)
    ax.set_ylabel("Latency (ms)", fontsize=40)

    ax.tick_params(axis='both', which='major', length=10, width=2)
    ax.tick_params(axis='both', which='major', labelsize=40)

    ax.set_ylim(-100, 430)
    ax.set_xlim(-100, 3500)

    # Compact Y-axis
    ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=3))
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.0f'))

    # Legend
    fig.legend(
        loc='upper center',
        ncol=1,
        bbox_to_anchor=(0.47, 1.4),
        bbox_transform=ax.transAxes,
        fontsize=40,
        frameon=False,
        columnspacing=0.5
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig("ARC_Latency.pdf", bbox_inches='tight')

    print("Plot saved as 'ARC_Latency.pdf'")
    plt.show()


# ------------------- Run -------------------
if __name__ == "__main__":
    arc_latencies = calculate_e2e_latency()
    plot_arc_latency(arc_latencies)
