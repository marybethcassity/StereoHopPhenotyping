from skimage import morphology as skmorph
import numpy as np
import open3d as o3d
import cv2

def dilate_erode(array):
    kernel = np.ones((5, 5), np.uint8)
    out = np.zeros(array.shape, dtype=np.uint8)
    
    for i in range(array.shape[2]):
        mask = array[:,:,i]
        mask_dilated = cv2.dilate(mask, kernel, iterations=1)
        mask_eroded = cv2.erode(mask_dilated, kernel, iterations=1)
        
        out[:,:,i] = mask_eroded
        
    for i in range(array.shape[0]):
        mask = out[i,:,:]
        mask_dilated = cv2.dilate(mask, kernel, iterations=1)
        mask_eroded = cv2.erode(mask_dilated, kernel, iterations=1)
        
        out[i,:,:] = mask_eroded
        
    out = np.clip(out,0,1)
    
    return out

def skeletonize(point_cloud):
    downpcd_np = np.asarray(point_cloud.points)
    max_x =  int(np.amax(downpcd_np[:,0])*1000)
    max_y = int(np.amax(downpcd_np[:,1])*1000)
    max_z = int(np.amax(downpcd_np[:,2])*1000)

    mask = np.zeros([max_x,max_y,max_z])
    for pt in downpcd_np:
        mask[int(pt[0]),int(pt[1]),int(pt[2])] = 1

    #skeleton = dilate_erode(mask)
    skeleton = skmorph.skeletonize_3d(mask.astype(bool))
    skeleton = skeleton.astype(np.uint8) * 255
    skeleton = np.argwhere(skeleton)#/1000
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mask)

    return downpcd_np