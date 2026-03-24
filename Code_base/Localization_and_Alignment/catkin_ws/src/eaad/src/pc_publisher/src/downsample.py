import open3d as o3d

# -----------------------------
# 1. Load the PCD file
# -----------------------------
pcd = o3d.io.read_point_cloud("Intersection_32_Beam1.pcd")  # replace with your file path
print("Original point cloud:")
print(pcd)
print(f"Number of points: {len(pcd.points)}")

# -----------------------------
# 2. Downsample using voxel grid
# -----------------------------
voxel_size = 0.5  # in meters, adjust for desired resolution
pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)

print("Downsampled point cloud:")
print(pcd_down)
print(f"Number of points: {len(pcd_down.points)}")

# -----------------------------
# 3. Save downsampled PCD
# -----------------------------
o3d.io.write_point_cloud("Intersection_32_Beam.pcd", pcd_down)

# -----------------------------
# 4. (Optional) Visualize
# -----------------------------
o3d.visualization.draw_geometries([pcd_down])
