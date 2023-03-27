import cv2
import os
import open3d as o3d

from config import *
from create_pointclouds import create_pointclouds
from register import register 
from cluster import cluster
from skeletonize import skeletonize
 
inPath = in_path
outPath = pc_path
registered = registered_path

def main(inPath, outPath, registered_path):

    count = 0

    if not (os.listdir(inPath)):

        print("creating point clouds :)")

        for image in os.listdir(inPath):
            
            side = image.split("0",1)[0]
            label = image.split("0",1)[1]
            num = "0" + label.split(".",1)[0] 
            
            if(side=="left"):
                count = count + 1
                if(count%20==0): #choose every 20th frame for good registration
                
                    left_im = cv2.imread(os.path.join(inPath,image))
                    right_im = cv2.imread(os.path.join(inPath,"right"+"0"+label))
                    
                    create_pointclouds(left_im, right_im, num, outPath)

    if not (os.path.isfile(registered_path)):       
         
        print("registering point clouds :)")

        register(voxel_size, pc_path, registered_path)
    
    clustered = cluster(registered_path)
    skel = skeletonize(clustered)

    o3d.visualization.draw_geometries([skel])

if __name__ == "__main__":
    main(inPath, outPath, registered_path)