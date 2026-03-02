import open3d as o3d
import numpy as np
from scipy.spatial import ConvexHull

def compute_mass_center(point_cloud):
    """
    Compute the center of mass (centroid) of a point cloud.
    """
    return np.mean(np.asarray(point_cloud.points), axis=0)

def compute_point_density(point_cloud):
    """
    Compute the point density: number of points divided by the convex hull area.
    """
    points = np.asarray(point_cloud.points)
    if len(points) < 4:  # ConvexHull requires at least 4 points in 3D
        return 0.0

    hull = ConvexHull(points)
    area = hull.area
    density = len(points) / area
    return density

def compute_l2_distance(center1, center2):
    """
    Compute the L2 distance between two 3D points.
    """
    return np.linalg.norm(center1 - center2)

def affinity_score(point_cloud1, point_cloud2, w1=1.0, w2=1.0):
    """
    Compute the affinity score between two point clouds.
    """
    # Compute mass centers
    center1 = compute_mass_center(point_cloud1)
    center2 = compute_mass_center(point_cloud2)
    
    # Compute L2 distance between mass centers
    l2_distance = compute_l2_distance(center1, center2)
    l2_inverse = 1 / l2_distance if l2_distance > 0 else float('inf')  # Avoid division by zero
    
    # Compute point densities
    density1 = compute_point_density(point_cloud1)
    density2 = compute_point_density(point_cloud2)
    
    # Compute absolute difference in point densities
    density_diff = abs(density1 - density2)
    
    # Affinity score
    score = w1 * l2_inverse + w2 * density_diff
    
    return score

# Example usage
if __name__ == "__main__":
    # Create two sample point clouds (representing a vehicle at two positions)
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()

    # Generate some random points for demonstration (replace with real point cloud data)
    points1 = np.random.rand(100, 3)  # 100 points for cloud 1
    points2 = np.random.rand(100, 3) + [2, 0, 0]  # Same points shifted in x-direction

    pcd1.points = o3d.utility.Vector3dVector(points1)
    pcd2.points = o3d.utility.Vector3dVector(points2)

    # Calculate affinity score
    score = affinity_score(pcd1, pcd2, w1=1.0, w2=0.01)
    print(f"Affinity Score: {score:.4f}")
