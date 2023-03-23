import cv2
import open3d as o3d
import numpy as np
import os

from config import *
from create_ply import create_ply

stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
    numDisparities = num_disp,
    blockSize = 3,
    uniquenessRatio = 5,
    speckleWindowSize = 5,
    speckleRange = 5,
    disp12MaxDiff = 1,
    P1 = 8*3*win_size**2,
    P2 =32*3*win_size**2)  

def create_pointclouds(left_im, right_im, num, out_path):
    
    disparity_map = stereo.compute(left_im, right_im)
    disparity_map = np.float32(np.divide(disparity_map,16))
        
    shape = np.shape(disparity_map)
    x = shape[0]
    y = shape[1]
    
    point_cloud = np.zeros([x,y,3])
    point_num = 0
        
    for i in range(x):
        for j in range(y):
            pc_x = (i - cx)*Tx / disparity_map[i,j]
            pc_y = (j - cy)*Tx / disparity_map[i,j]
            pc_z = (f*Tx)/disparity_map[i,j]
            point_cloud[i,j,0] = pc_x
            point_cloud[i,j,1] = pc_y
            point_cloud[i,j,2] = pc_z
            
    hsv = cv2.cvtColor(left_im, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(left_im, cv2.COLOR_BGR2RGB)
        
    #mask by hue 
        
    lower_green = np.array([35,30,30])
    upper_green = np.array([70,255,255])
    mask_hue = cv2.inRange(hsv, lower_green, upper_green)
        
    mask_map = disparity_map > disparity_map.min()
    point_cloud_z = point_cloud[:,:,2]
    point_cloud_x = point_cloud[:,:,0]
    
    mask = np.logical_and(mask_map,mask_hue)
         
    output_points = point_cloud[mask]
    output_colors = rgb[mask]

    output = np.hstack([output_points, output_colors])
            
    output_file = os.path.join(out_path,num+".ply")
    create_ply(output, output_file)
        
    pcd = o3d.io.read_point_cloud(output_file)
    pcd_down = pcd.uniform_down_sample(every_k_points=5)
    cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    cl2, ind = cl.remove_radius_outlier(nb_points=16, radius=0.05)
    o3d.io.write_point_cloud(output_file, cl2)



