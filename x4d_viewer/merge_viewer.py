# -*- coding: utf-8 -*-
"""
Created on Sun Sep 30 22:20:03 2018

@author: genki
"""
import sys
from time import sleep
import os
import numpy as np
import cv2
import open3d
import time
import cv2
import sys
import json
import linecache
import shutil

class X4D():
    

    def __init__(self,frame_num_):
        self.frame_num = frame_num_
        print("create X4D ")
        self.point_cloud_files = []
        self.image_files = []
        self.timestamps = []
        self.cam_poses = []
        self.images = []
        self.point_clouds = []

    def outputPointcloudFiles(self):
        print(self.point_cloud_files)
    

def read_x4d_file(x4d_filename):
    
    print("reading x4d file from ",x4d_filename)
    
    tmp = x4d_filename.split("\\")[:-1]
    base_folder = ""
    for i in range(len(tmp)):
        base_folder += tmp[i]+"/"
    print(base_folder)
    
    
    f = open(x4d_filename,"r")
    strings = f.readlines()
    
    # frame num
    string = strings[0]
    frame_num = int(string.split(" ")[-1])
    print("frame number is ",frame_num)
    strings.pop(0)
    
    # 
    x4d = X4D(frame_num)
    #x4d.frame_num = frame_num
    
    while(True):
        string = strings[0][:-1]
        strings.pop(0)
        if (len(strings) == 0):
            print("cannot read x4d file")
            return

        # Data or Atribute
        print(string)
        if (string[:-1] == "Data"):
            break
        elif(string[:-1] == "Attribute"):
            print("Attribute : ")
            while(True):
                string = strings.pop(0)[:-1]
                if (string == "}"):
                    break
                print(string)
        
           
    word = string[:-1]
    print("read : ",word)
    Data_structures = []
    if (word == "Data"):
        while(True):
            string = strings[0][:-1]
            strings.pop(0)
            if (string == "}"):
                break
            Data_structures.append(string)
        print("Data sturecure is ")
        print(Data_structures)
    
    # read datas
    for t in range(len(Data_structures)):
        types = Data_structures[t]
        text_list = []
        for i in range(frame_num):
            string = strings.pop(0)[:-1]
            text_list.append(string)
        #print(text_list)
        if (types == "ply"):
            for i in range(frame_num):
                #x4d.image_files = text_list
                x4d.point_cloud_files.append(base_folder+text_list[i])
        elif (types == "color_image"):
            for i in range(frame_num):
                #x4d.image_files = text_list
                x4d.image_files.append(base_folder+text_list[i])
        elif (types == "timestamp"):
            for i in range(frame_num):
                x4d.timestamps.append(int(text_list[i]))
    return x4d

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

# main
print("argv[1] : x4d file 0")
print("argv[2] : x4d file 1")
print("argv[3] : rotate mat")
print("arfv[4] : trans")

x4d_file_name0 = sys.argv[1]
x4d_file_name1 = sys.argv[2]
rotate_mat_file = sys.argv[3]
trans_file = sys.argv[4]

#x4d_0 = X4D()
#x4d_1 = X4D()

x4d_0 = read_x4d_file(x4d_file_name0)
x4d_0.outputPointcloudFiles()
x4d_1 = read_x4d_file(x4d_file_name1)
x4d_1.outputPointcloudFiles()

frame_num = x4d_0.frame_num

# read image and point cloud

print("read image and point cloud")
for i in range(frame_num):
    image = cv2.imread(x4d_0.image_files[i],1)
    x4d_0.images.append(image)
    
    point_cloud = open3d.io.read_point_cloud(x4d_0.point_cloud_files[i])
    point_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    point_cloud.orient_normals_towards_camera_location()
    #point_cloud = point_cloud.scale(0.001,center=(0,0,0))
    x4d_0.point_clouds.append(point_cloud)
    cv2.imshow("read image",image)
    cv2.waitKey(1)
cv2.destroyAllWindows()

print("read image and point cloud")
for i in range(frame_num):
    image = cv2.imread(x4d_1.image_files[i],1)
    x4d_1.images.append(image)
    
    point_cloud = open3d.io.read_point_cloud(x4d_1.point_cloud_files[i])
    point_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    point_cloud.orient_normals_towards_camera_location()
    #point_cloud = point_cloud.scale(0.001,center=(0,0,0))
    open3d.io.write_point_cloud("test.ply",point_cloud)
    x4d_1.point_clouds.append(point_cloud)
    cv2.imshow("read image",image)
    cv2.waitKey(1)
cv2.destroyAllWindows()

rmat = np.loadtxt(rotate_mat_file)
trans = np.loadtxt(trans_file)
rtmat = np.eye(4)
rtmat[0:3,0:3] = rmat
rtmat[0:3,3] = trans*1000
rtmat = np.linalg.inv(rtmat)

print("rt mat ")
print(rtmat)

# icp
import copy

voxel_radius = [0.04, 0.02, 0.01]
max_iter = [50, 30, 14]
current_transformation = np.identity(4)

for scale in range(3):
    

    iter = max_iter[scale]
    radius = voxel_radius[scale]*1000
    print([iter, radius, scale])


    pc_new_0 = x4d_0.point_clouds[0]
    pc_new_1 = x4d_1.point_clouds[0]
    pc_new_1 = copy.deepcopy(pc_new_1).transform(rtmat)
    source_down = pc_new_0.voxel_down_sample(radius)
    target_down = pc_new_1.voxel_down_sample(radius)
    #pc_new_0 = pc_new_0.paint_uniform_color([1, 0.706, 0])
    #pc_new_1 = pc_new_1.paint_uniform_color([0, 0.706, 1])
    source_down.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    #icp
    result_icp = open3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            open3d.pipelines.registration.TransformationEstimationForColoredICP(),
            open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
    icp_rtmat = result_icp.transformation
    current_transformation = result_icp.transformation
    print(result_icp)
    '''
    threshold = 0.05*1000
    trans_init = np.eye(4)
    draw_registration_result(pc_new_0, pc_new_1, trans_init)
    reg_p2p = open3d.pipelines.registration.registration_icp(
        pc_new_0, pc_new_1, threshold, trans_init,
        open3d.pipelines.registration.TransformationEstimationPointToPoint())
        #open3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_rtmat = reg_p2p.transformation
    print(icp_rtmat)
    threshold = 0.01*1000
    reg_p2p = open3d.pipelines.registration.registration_icp(
        pc_new_0, pc_new_1, threshold, icp_rtmat,
        open3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(pc_new_0, pc_new_1, icp_rtmat)
    '''
    print(icp_rtmat)
    #pc_new_0 = copy.deepcopy(pc_new_0).transform(icp_rtmat)
    #reg_p2l = open3d.pipelines.registration.registration_icp(
    #    pc_new_0, pc_new_1, threshold, trans_init,
    #    open3d.pipelines.registration.TransformationEstimationPointToPlane())
    #print(reg_p2p)
    #x4d_0.point_clouds[frame_index] = pc_new_0
    #x4d_1.point_clouds[frame_index] = pc_new_1

# show the point cloud
cur_time = time.time()
pre_time = time.time()

# visualized window
vis = open3d.visualization.VisualizerWithKeyCallback()
vis.create_window(
    window_name="visualilzer",
    width = 800,
    height = 800,
    left = 50,
    top = 50)
pc_0 = open3d.geometry.PointCloud()
pc_1 =  open3d.geometry.PointCloud()
vis.add_geometry(pc_0)
vis.add_geometry(pc_1)
#view_control = vis.get_view_control()
view_control_1 = vis.get_view_control()


param_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()
param_pre_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()

#render_option_1 = vis.get_render_option()
render_option_1 = vis.get_render_option()




pre_time = time.time()*10**6
frame_index = 0
while True:
    

    if time.time()*10**6 - pre_time + x4d_0.timestamps[frame_index] > x4d_0.timestamps[frame_index + 1]:
        frame_index += 1
        print("frame ",frame_index)
        pc_new_0 = x4d_0.point_clouds[frame_index]
        pc_new_1 = x4d_1.point_clouds[frame_index]
        pc_new_1 = copy.deepcopy(pc_new_1).transform(rtmat)
        pc_new_0 = copy.deepcopy(pc_new_0).transform(icp_rtmat)
        #pc_new_0 = pc_new_0.paint_uniform_color([1, 0.706, 0])
        #pc_new_1 = pc_new_1.paint_uniform_color([0, 0.706, 1])

        #icp
        #threshold = 0.1
        #trans_init = np.eye(4)
        #reg_p2p = open3d.pipelines.registration.registration_icp(
        #    pc_new_0, pc_new_1, threshold, trans_init,
        #    open3d.pipelines.registration.TransformationEstimationPointToPoint())
        #icp_rtmat = reg_p2p.transformation
        #pc_new_0 = pc_new_0.transform(icp_rtmat)
        #reg_p2l = open3d.pipelines.registration.registration_icp(
        #    pc_new_0, pc_new_1, threshold, trans_init,
        #    open3d.pipelines.registration.TransformationEstimationPointToPlane())
        #print(reg_p2p)
        #pc_new_0.paint_uniform_color((1,0.0))
        #pc_new_1.paint_uniform_color((0,0,1))
        vis.remove_geometry(pc_0)
        vis.remove_geometry(pc_1)
        
        pc_0 = pc_new_0        
        pc_1 = pc_new_1
        vis.add_geometry(pc_0)
        vis.add_geometry(pc_1)
        
        pre_time = time.time()*10**6
        
        if (frame_index + 1 == x4d_0.frame_num):
            frame_index = 0
        
        # 多分ポインタを取得している
    view_control_1 = vis.get_view_control()
   
    # 前のパラメータを設定
    view_control_1.convert_from_pinhole_camera_parameters(param_pre_1)
    

    render_option_1 = vis.get_render_option()
    render_option_1.load_from_json("vis_set_1.json")
   
    #更新
    vis.update_geometry(pc_0)
    vis.poll_events()
    vis.update_renderer()
    vis.update_geometry(pc_1)
    #vis.update_geometry([pc_0,pc_1])
    vis.poll_events()
    vis.update_renderer()
    view_control_1 = vis.get_view_control()
    param_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()

    

'''


# visualize window setting





# define call bacl func
paint_mode = 0
pre_time = 0



pc_ref = open3d.geometry.PointCloud()


vis_ref.add_geometry(pc_ref)


transform_matrix = np.zeros([4,4])
rotate_euler = np.zeros([3])
trans = np.zeros([3])
trans[2] += 1.0


#view_control = vis.get_view_control()
view_control_1 = vis_ref.get_view_control()


param_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()
param_pre_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()

#render_option_1 = vis.get_render_option()
render_option_1 = vis_ref.get_render_option()


save_num = 0
white = np.zeros([3])
white[0] = 255
white[1] = 255
white[2] = 255


render_option_1.save_to_json("vis_set_1.json")



while True:
    

    if (pre_time != os.stat(reference_file).st_mtime_ns):
        sleep(0.04)
        print("renew")
        pc_new1 = open3d.io.read_point_cloud(reference_file)
       

        vis_ref.remove_geometry(pc_ref)
        
	#vis.remove_geometry(pc)
        pc_ref = pc_new1
        sphere = open3d.geometry.TriangleMesh.create_sphere(radius = 0.01)
        sphere.translate([0.0,0.0,0.0])
        sphere.paint_uniform_color([1.0,0.0,0.0])
        vis_ref.add_geometry(sphere)
        
        vis_ref.add_geometry(pc_ref)
        
        

        #vis.add_geometry(pc)
        #display_inlier_outlier(pc, ind)
        pre_time = os.stat(reference_file).st_mtime_ns
        # 多分ポインタを取得している
    view_control_1 = vis_ref.get_view_control()
   
    # 前のパラメータを設定
    view_control_1.convert_from_pinhole_camera_parameters(param_pre_1)
    

    render_option_1 = vis_ref.get_render_option()
    
    render_option_1.load_from_json("vis_set_1.json")
   
    #更新
    vis_ref.update_geometry(pc_ref)
    vis_ref.poll_events()
    vis_ref.update_renderer()
    view_control_1 = vis_ref.get_view_control()
    param_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()
'''
