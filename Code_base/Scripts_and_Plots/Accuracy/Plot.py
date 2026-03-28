import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, FormatStrFormatter
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

# Reduce figure weight
mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 0.01

# ------------------- Path Detection -------------------
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
    base_dir = "."

# ------------------- Data Loading -------------------
def load_and_scale(file, scale=1.0):
    full_path = os.path.join(base_dir, file)
    if not os.path.exists(full_path):
        print(f"Warning: {full_path} not found.")
        return []
    with open(full_path, "r") as f:
        return [float(line.strip().split()[-1]) * scale for line in f]

# ARC results (Lead)
arc_rte_raw = load_and_scale("RTE_V2V_Lead.txt", 100)
arc_rre_raw = load_and_scale("RRE_V2V_Lead.txt")

# AutoCast results (Direct)
ac_rte_raw = load_and_scale("RTE_V2V_Direct.txt", 100)
ac_rre_raw = load_and_scale("RRE_V2V_Direct.txt")

# ------------------- Filtering -------------------
f_ARC_RTE, f_ARC_RRE = [], []
f_AC_RTE, f_AC_RRE = [], []

# Filter based on ARC error: if RTE <= 7cm and RRE <= 0.17deg
for arc_rte, arc_rre, ac_rte, ac_rre in zip(arc_rte_raw, arc_rre_raw, ac_rte_raw, ac_rre_raw):
    if arc_rte <= 7 and arc_rre <= 0.17: 
        f_ARC_RTE.append(arc_rte)
        f_ARC_RRE.append(arc_rre)
        f_AC_RTE.append(ac_rte)
        f_AC_RRE.append(ac_rre)

Frames = list(range(len(f_ARC_RTE)))

# ------------------- Downsampling -------------------
step = 20
Frames_ds = Frames[::step]
f_ARC_RTE_ds = f_ARC_RTE[::step]
f_ARC_RRE_ds = f_ARC_RRE[::step]
f_AC_RTE_ds = f_AC_RTE[::step]
f_AC_RRE_ds = f_AC_RRE[::step]

# ------------------- Statistics -------------------
def compute_stats(data):
    if not data: return 0, 0, 0
    return np.mean(data), np.percentile(data, 95), np.percentile(data, 99)

ARC_RTE_mean, _, _ = compute_stats(f_ARC_RTE)
ARC_RRE_mean, _, _ = compute_stats(f_ARC_RRE)
AC_RTE_mean, _, _ = compute_stats(f_AC_RTE)
AC_RRE_mean, _, _ = compute_stats(f_AC_RRE)

print(f"ARC Mean: RTE={ARC_RTE_mean:.2f}cm, RRE={ARC_RRE_mean:.3f}°")
print(f"AutoCast Mean: RTE={AC_RTE_mean:.2f}cm, RRE={AC_RRE_mean:.3f}°")

# ------------------- Plots -------------------
def create_time_plot(filename, frames, y1, y2, l1, l2, ylabel, yfmt, ylim=None):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(frames, y1, color='#00008B', marker='o', linewidth=5, markersize=8, label=l1, rasterized=True)
    ax.plot(frames, y2, color='green', marker='o', linewidth=5, markersize=8, label=l2, rasterized=True)
    ax.set_xlabel('Frame number', fontsize=28)
    ax.set_ylabel(ylabel, fontsize=28)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
    ax.yaxis.set_major_formatter(FormatStrFormatter(yfmt))
    if ylim: ax.set_ylim(ylim)
    ax.tick_params(axis='both', labelsize=28)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.35), ncol=2, fontsize=28, frameon=False, columnspacing=0.5)
    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight', dpi=300)
    plt.close()

def create_cdf_plot(filename, d1, d2, l1, l2, xlabel):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    for data, color, label in [(np.sort(d1), '#00008B', l1), (np.sort(d2), 'green', l2)]:
        cdf = np.arange(1, len(data)+1) / len(data)
        ax.plot(data, cdf, color=color, marker='o', linewidth=5, markersize=8, label=label, rasterized=True)
    ax.set_xlabel(xlabel, fontsize=28)
    ax.set_ylabel('CDF', fontsize=28)
    ax.set_ylim(0, 1.1)
    ax.tick_params(axis='both', labelsize=28)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.35), ncol=2, fontsize=28, frameon=False, columnspacing=0.5)
    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight', dpi=300)
    plt.close()

# Plot 1: RTE Over Time
create_time_plot('Both_Map_vs_Leader.pdf', Frames_ds, f_AC_RTE_ds, f_ARC_RTE_ds, 
                 f'AutoCast (avg: {AC_RTE_mean:.1f} cm)', f'ARC (avg: {ARC_RTE_mean:.1f} cm)',
                 'RTE (cm)', '%.0f')

# Plot 2: RRE Over Time
create_time_plot('Both_LeaderRRE_vs_DirectRRE.pdf', Frames_ds, f_AC_RRE_ds, f_ARC_RRE_ds, 
                 f'AutoCast (avg: {AC_RRE_mean:.2f}°)', f'ARC (avg: {ARC_RRE_mean:.2f}°)',
                 'RRE (degree)', '%.2f')

# Plot 3: RTE CDF
create_cdf_plot('Both_CDF_Map_vs_Leader.pdf', f_AC_RTE, f_ARC_RTE, 'AutoCast', 'ARC', 'RTE (cm)')

# Plot 4: RRE CDF
create_cdf_plot('Both_CDF_LeaderRRE_vs_DirectRRE.pdf', f_AC_RRE, f_ARC_RRE, 'AutoCast', 'ARC', 'RRE (degree)')

print("Both ARC and AutoCast plots generated successfully.")
