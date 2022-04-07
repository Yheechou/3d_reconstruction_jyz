import open3d as o3d
import numpy as np
import copy
from time import *
import math

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    o3d.geometry.estimate_normals(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, theta, source, target):

    trans_init = np.asarray([[np.cos(theta), 0.0, np.sin(theta), 0.0], 
                             [0.0, 1.0, 0.0, 0.0],
                             [-np.sin(theta), 0.0, np.cos(theta), 0.0], 
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                               target_fpfh, voxel_size):
   distance_threshold = voxel_size * 1.5
   # print(":: RANSAC registration on downsampled point clouds.")
   # print("   Since the downsampling voxel size is %.3f," % voxel_size)
   # print("   we use a liberal distance threshold %.3f." % distance_threshold)
   result = o3d.registration.registration_ransac_based_on_feature_matching(
       source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
       o3d.registration.TransformationEstimationPointToPoint(False), 4, [
           o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
           o3d.registration.CorrespondenceCheckerBasedOnDistance(
               distance_threshold)
       ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
   return result

if __name__ == "__main__":

    voxel_size = 0.1 # means 5cm for this dataset
    fitness_door = 0.85


    # 3D Reconstruction
    file_num = [7, 4, 4, 3, 3, 4, 2] # now at 2_1 deleted

    room_dict = [[0,1],[2,3,4],[5,6,7],[8,9,10],[11,12],[13,14,15],[],[16,17,18],[19,20,21],[22,23],[24,25,26]]
    room_dict2 = [[0,1,2,3,4,5,6],[7,8,9,10],[11,12,13,14],[15,16,17],[18,19,20],[21,22,23,24],[25,26]]
    room_num = 0
    room = []

    # load the initial ply : record in room[0:26]
    for i in range(0,7): # i  for i_room_x: the same camera
        for j in range(0,file_num[i]): # j for x_room_j: the different room
            room.append(o3d.io.read_point_cloud("to_yunqi_offline_data/ipark_room_ply/" + str(i) + "_room_" + str(j) + ".ply"))
            room_num = room_num + 1
    
    room_num = 2 # 0 for room0_0, 1 for room0_1
    flag = 1
    # start to handle the room
    for i in range(1,7): # i  for i_room_x: the same camera
        print("-----------------Group"+str(i)+"'s Matching---------------------")
        # break
        flag = 1
        for j in range(0,file_num[i]): # j for x_room_j: the different room

            if flag==0:
                break
            source = room[room_dict2[i][j]]
            for k in range(0,i):
                if flag==0:
                    break
                for z in range(0,file_num[k]): 
                    print("Now we compare "+str(i)+"_room_"+str(j)+" with "+str(k)+"_room_"+str(z))
                    target = room[room_dict2[k][z]]
                    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, math.pi*0, source, target)
                    result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
                    print(result_ransac.fitness)
                    
                    # success correspondence
                    if i==5 and j==0 and k==1 and z==1:
                        continue
                    if i==5 and j==0 and k==4 and z==2:
                        continue
                    if i==5 and j==1 and k==1 and z==2:
                        continue                    
                    if result_ransac.fitness > fitness_door:
                        for tmp in range(0,file_num[i]):
                            room[room_dict2[i][tmp]].transform(result_ransac.transformation)
                        print("******** Success correspondenc "+str(i)+"_room_"+str(j)+" with "+str(k)+"_room_"+str(z)+"***********")
                        flag = 0
                        break

    o3d.visualization.draw_geometries([room[0], room[1], room[2], room[3], room[4], room[5], room[6], room[7], room[8], room[9], room[10], room[11], room[12], room[13], room[14], room[15], room[16], room[17], room[18], room[19], room[20], room[21], room[22], room[23], room[24], room[25],room[26]])
