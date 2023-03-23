def create_ply(data, filename):
    data = data.reshape(-1,6)
    #vertices = np.hstack([vertices.reshape(-1,3),colors])
    
    ply_header = '''ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header
        '''
    
    with open(filename, 'w') as f:
        f.write(ply_header %dict(vert_num=len(data)))
        np.savetxt(f,data,'%f %f %f %d %d %d')
    
