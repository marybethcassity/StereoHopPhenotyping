#import matplotlib.pyplot as plt
#import open3d as o3d
import cv2
import os

from config import *
from create_pointclouds import create_pointclouds
from register import register 
 
inPath = in_path
outPath = pc_path
registered = registeted_path

def main(inPath, outPath):

    count = 0

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
                
    register(voxel_size, pc_path, registeted_path)
    
if __name__ == "__main__":
    main(inPath, outPath)