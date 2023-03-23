stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
    numDisparities = num_disp,
    blockSize = 3,
    uniquenessRatio = 5,
    speckleWindowSize = 5,
    speckleRange = 5,
    disp12MaxDiff = 1,
    P1 = 8*3*win_size**2,
    P2 =32*3*win_size**2)  

fx = 1054.7600
fy = 1054.4700
cx = 956.0000
cy = 570.3380
f = fx + fy / 2
Tx = 0.1197760

win_size = 5
min_disp = 25
max_disp = 281
num_disp = max_disp - min_disp