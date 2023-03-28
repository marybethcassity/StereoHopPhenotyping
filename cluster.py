import os
import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import kneed
import matplotlib.pyplot as plt

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

def dbscan(pointcloud,dist,neighbors):
    
    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
        pointcloud.cluster_dbscan(eps=dist, min_points=6, print_progress=True))

    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pointcloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    """""
    The following code is to select by largest cluster.
    """

    #unique_labels = np.unique(labels)
    #unique_labels = unique_labels[unique_labels >= 0]
    #largest_cluster_label = np.argmax(np.bincount(unique_labels))
    #mask = labels == largest_cluster_label
    #selected_points = pointcloud.select_by_index(np.where(mask)[0])

    """""
    The following code is to select clusters using elbow method.
    """

    unique, counts = np.unique(labels, return_counts = True)
    sorted_counts = np.sort(counts, axis = 0)
    i = np.arange(len(sorted_counts))
    knee = kneed.KneeLocator(i, sorted_counts, curve='convex', direction='increasing', online = False)
    indeces = np.where(counts > sorted_counts[knee.knee])
    selected_labels = unique[indeces]
    selected_labels = np.array(selected_labels)  
    selected_labels = selected_labels[selected_labels != -1]
    mask = np.isin(labels, selected_labels)
    selected_points = pointcloud.select_by_index(np.where(mask)[0])
    
    return selected_points

def findeps(X):
    neigh = NearestNeighbors(n_neighbors=6)
    nbrs = neigh.fit(X)
    distances, indeces = nbrs.kneighbors(X)
    distances = np.sort(distances, axis=0)
    distances = distances[:,1]
    convolved = moving_average(distances,5)
    i = np.arange(len(convolved))
    knee = kneed.KneeLocator(i, convolved, curve='convex', direction='increasing', online = False)
    x = distances[knee.knee]
    
    return x

def cluster(registered_path):
    pcd = o3d.io.read_point_cloud(registered_path)
    downpcd = pcd.voxel_down_sample(voxel_size=0.01)
    downpcd_np = np.asarray(downpcd.points)
    eps = findeps(downpcd_np)
    filtered = dbscan(downpcd, eps, 6)

    return filtered
