from json.encoder import INFINITY
import numpy as np
from tqdm import tqdm
import open3d as o3d
import random as rand
import copy

mesh = o3d.io.read_triangle_mesh("WorkCell/parts/cylinder.stl")
pointcloud = mesh.sample_points_poisson_disk(100000)

# you can plot and check
o3d.visualization.draw_geometries([mesh])
o3d.visualization.draw_geometries([pointcloud])


## init models
scene = o3d.io.read_point_cloud("PointClouds/scene.pcd")
obj_local = o3d.io.read_point_cloud("PointClouds/object-local.pcd")
obj_global = o3d.io.read_point_cloud("PointClouds/object-global.pcd")

## init random number generator
rand.seed()

##o3d.visualization.draw_geometries([obj_global, scene], window_name='Before global alignment')


## init feature discriptos (surface normal)

# Compute surface normals
obj_global.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(11))
scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(11))

## Find feature matches
corr = o3d.utility.Vector2iVector()
obj_f = o3d.pipelines.registration.compute_fpfh_feature(obj_global, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
scene_f = o3d.pipelines.registration.compute_fpfh_feature(scene, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))

## cast as np arrays for later (transpose for convienence)
obj_f = np.asarray(obj_f.data).T
scene_f = np.asarray(scene_f.data).T


for i in tqdm(range(obj_f.shape[0]), desc = "Calculating feature distnaces"):
    fobj = obj_f[i]
    dist = np.sum((fobj - scene_f)**2, axis=-1)
    dist_min = np.argmin(dist)
    corr.append((i, dist_min))


## init KD tree and Transformation estimator
tree = o3d.geometry.KDTreeFlann(scene)

## init highest numbver of inliers
inliers_best = 0

## Set threshold
thres = 0.0001

## Set RANSAC iterations
ransacIt = 2000

## initialize root mean square error as infinite
rmse = INFINITY

## Empty so it can be updated when a geuss is made
transform = []


## RANSAC N times and show progress (tqdm)
for i in tqdm(range(ransacIt), desc='RANSAC'):   
    
    ## pick 3 random correspondances
    corri = o3d.utility.Vector2iVector(rand.choices(corr, k=3))
    
    ## Copy global oject to transform
    obj_global_temp = copy.deepcopy(obj_global)

    tepp = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    
    ## calculate transform using Open3D
    transform_temp = tepp.compute_transformation(obj_global_temp, scene, corri)
    
    ## Test transformation of obj on a copy
    obj_global_temp.transform(transform_temp)
    
    
    ## Find inliers that meet threshold
    inliers = 0
    for j in range(len(obj_global_temp.points)):
        k, idx, dist = tree.search_knn_vector_3d(obj_global_temp.points[j], 1)
        
        ## Note: dist is squared when mathcing so threshold is for dist^2
        if dist[0] <= thres:
            inliers += 1


    ## check if more inliers were found, update if necessary
    if inliers > inliers_best:
        inliers_best = inliers
        transform = transform_temp

        ## find RMSE for quality assurance
        rmse = tepp.compute_rmse(obj_global_temp, scene, corr)
 


# Print pose
print("RANSAC transformation: ")
print(transform)
print("RANSAC rmse valuse :",rmse)
obj_global.transform(transform)
o3d.visualization.draw_geometries([obj_global,scene])