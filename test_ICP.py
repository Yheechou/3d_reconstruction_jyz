import open3d as o3d
import numpy as np

#读取电脑中的 ply 点云文件
source = o3d.io.read_point_cloud("/Users/apple/Desktop/3D室内重建/project2/scannet_20/scene0403_00/scene0403_00_vh_clean.ply")  #source 为需要配准的点云
target = o3d.io.read_point_cloud("/Users/apple/Desktop/3D室内重建/project2/scannet_20/scene0403_01/scene0403_01_vh_clean.ply")  #target 为目标点云
print(source)
print(target)

print("Downsample the point cloud with a voxel of 0.05")
source = o3d.geometry.voxel_down_sample(source,voxel_size=0.1)
target = o3d.geometry.voxel_down_sample(target,voxel_size=0.1)
print(source)
print(target)

#为两个点云上上不同的颜色
source.paint_uniform_color([1, 0.706, 0])    #source 为黄色
target.paint_uniform_color([0, 0.651, 0.929])#target 为蓝色

#为两个点云分别进行outlier removal
processed_source, outlier_index = o3d.geometry.radius_outlier_removal(source,
                                              nb_points=16,
                                              radius=0.5)

processed_target, outlier_index = o3d.geometry.radius_outlier_removal(target,
                                              nb_points=16,
                                              radius=0.5)
threshold = 1.0  #移动范围的阀值
trans_init = np.asarray([[1,0,0,0],   # 4x4 identity matrix，这是一个转换矩阵，
                         [0,1,0,0],   # 象征着没有任何位移，没有任何旋转，我们输入
                         [0,0,1,0],   # 这个矩阵为初始变换
                         [0,0,0,1]])

#运行icp
reg_p2p = o3d.registration.registration_icp(
        processed_source, processed_target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())

#将我们的矩阵依照输出的变换矩阵进行变换
print(reg_p2p)
processed_source.transform(reg_p2p.transformation)

#将变换的矩阵存储下来
#o3d.io.write_point_cloud("source.ply",processed_source)
#o3d.io.write_point_cloud("target.ply",processed_target)

#创建一个 o3d.visualizer class
vis = o3d.visualization.Visualizer()
vis.create_window()

#将两个点云放入visualizer
vis.add_geometry(processed_source)
vis.add_geometry(processed_target)

#让visualizer渲染点云
vis.update_geometry()
vis.poll_events()
vis.update_renderer()

vis2 = o3d.visualization.Visualizer()
vis2.create_window()
vis2.add_geometry(source)
vis2.add_geometry(target)
vis2.update_geometry()
vis2.poll_events()
vis2.update_renderer()

vis2.run()
vis.run()

