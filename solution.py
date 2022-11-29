import open3d as o3d
import numpy as np
from tqdm import tqdm
import random

it = 1000
thressq = 0.01**2

# Load
obj = o3d.io.read_point_cloud('PointClouds/object-global.pcd')
colors = np.zeros_like(obj.points)
colors[:,0] = 255
obj.colors = o3d.utility.Vector3dVector(colors)
scn = o3d.io.read_point_cloud('PointClouds/scene.pcd')

# Show
o3d.visualization.draw_geometries([obj, scn], window_name='Before global alignment')

# Compute surface normals
obj.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(11))
scn.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(11))

# Compute shape features
obj_features = o3d.pipelines.registration.compute_fpfh_feature(obj, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
scn_features = o3d.pipelines.registration.compute_fpfh_feature(scn, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))

obj_features = np.asarray(obj_features.data).T
scn_features = np.asarray(scn_features.data).T

# Find feature matches
corr = o3d.utility.Vector2iVector()
for j in tqdm(range(obj_features.shape[0]), desc='Correspondences'):
    fobj = obj_features[j]
    dist = np.sum((fobj - scn_features)**2, axis=-1)
    kmin = np.argmin(dist)
    corr.append((j, kmin))

# Create a k-d tree for scene
tree = o3d.geometry.KDTreeFlann(scn)

# Start RANSAC
#random.seed(123456789)
random.seed()
inliers_best = 0
for i in tqdm(range(it), desc='RANSAC'):   
    # Sample 3 random correspondences
    corri = o3d.utility.Vector2iVector(random.choices(corr, k=3))
    
    # Estimate transformation
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = est.compute_transformation(obj, scn, corri)
    
    # Apply pose
    obj_aligned = o3d.geometry.PointCloud(obj)
    obj_aligned.transform(T)
    
    # Validate
    inliers = 0
    for j in range(len(obj_aligned.points)):
        k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
        if dist[0] < thressq:
            inliers += 1

    # Update result
    if inliers > inliers_best:
        #print(f'Got a new model with {inliers}/{len(obj_aligned.points)} inliers!')
        inliers_best = inliers
        pose = T

# Print pose
print('Got the following pose:')
print(pose)

# Show result
o3d.visualization.draw_geometries([obj.transform(pose), scn], window_name='After global alignment')
