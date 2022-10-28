import numpy as np
import struct
import open3d

def read_bin_velodyne(path):
    '''read bin file and transfer to array data'''
    pc_list=[]
    with open(path,'rb') as f:
        content=f.read()
        pc_iter=struct.iter_unpack('ffff',content)
        for idx,point in enumerate(pc_iter):
            pc_list.append([point[0],point[1],point[2]])
    return np.asarray(pc_list,dtype=np.float32)

def main():
    pc_path='/home/thinking/detection_ws/OpenPCDet/data/nuscenes/v1.0-mini/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin'
    # example=read_bin_velodyne(pc_path)
    example = np.fromfile(pc_path, dtype=np.float32, count=-1).reshape(-1, 5)
    example_xyz=example[:,:3]
    example_xyz=example_xyz[example_xyz[:,2]>-3]

    # From numpy to Open3D
    pcd = open3d.open3d.geometry.PointCloud()
    pcd.points= open3d.open3d.utility.Vector3dVector(example_xyz)
    vis_ = open3d.visualization.Visualizer()
    vis_.create_window()
    vis_.add_geometry(pcd)
    render_options = vis_.get_render_option()
    render_options.point_size = 1
    render_options.background_color = np.array([0, 0, 0])
    vis_.run()
    vis_.destroy_window()
    # pcd.points= open3d.open3d.utility.Vector3dVector(example)
    # open3d.open3d.visualization.draw_geometries([pcd])

if __name__=="__main__":
    main()



