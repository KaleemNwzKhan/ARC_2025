import numpy as np
import sys
def read_cell_count(file_path):
    with open(file_path, 'r') as f:
        return [int(line.strip()) for line in f]

def read_matrices(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    matrices = []
    for i in range(0, len(lines), 4):  # Each matrix has 4 rows
        matrix = np.array([list(map(float, lines[i + j].split())) for j in range(4)])
        matrices.append(matrix)
    return matrices

def write_asp_file(cell_counts, matrices_ndt, matrices_align, output_file):
    with open(output_file, 'w') as f:
        for i, count in enumerate(cell_counts):
            matrix = matrices_ndt[i] if count < 20 else matrices_align[i]
            for row in matrix:
                f.write(" ".join(map(str, row)) + "\n")

# File paths

cell_count_file= sys.argv[1]
ndt_file= sys.argv[3]
align_file = sys.argv[2]
output_file= sys.argv[4]
#cell_count_file = "Cells_count.txt"
#ndt_file = "Lead_Vehicle_NDT.txt"
#align_file = "Direct_Alignment_Poses.txt"
#output_file = "ASP.txt"

# Read data
cell_counts = read_cell_count(cell_count_file)
matrices_ndt = read_matrices(ndt_file)
matrices_align = read_matrices(align_file)

# Write output file
write_asp_file(cell_counts, matrices_ndt, matrices_align, output_file)

