# StereoHopPhenotyping
Create point clouds using OpenCV SGBM and ICP registration from Open3D. Densely sample images for good registration. Currently masking images by hue. Working on semantic segmentation with PyTorch. 

Jupyter Notebooks:
In automatePointclouds.ipynb, update parameters in 3rd block.

Python:
Update parameters in config.py 

A good explanation of parameters can be found at https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html. ImageJ https://imagej.nih.gov/ij/download.html is an easy way to estimate minimum and maximum disparity. fx, fy, cx, cy, and Tx are intrinsic to the stereo camera. 
