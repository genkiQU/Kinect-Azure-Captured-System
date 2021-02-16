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
    frame_num = 0
    point_cloud_files = []
    image_files = []
    timestamps = []
    cam_poses = []
    images = []
    point_clouds = []

    def __init__(self,frame_num_):
        self.frame_num = frame_num_
       
        
        
    def outputPointcloudFiles(self):
        print(self.point_cloud_files)
    

def read_x4d_file(x4d_filename):
    
    print("reading x4d file from ",x4d_filename)
    
    tmp = x4d_file_name.split("\\")[:-1]
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
    
    #x4d.outputPointcloudFiles()
        
    return x4d

# main
print("argv[1] : x4d file")

x4d_file_name = sys.argv[1]

print("argv[1] : x4d file ",x4d_file_name)


x4d = read_x4d_file(x4d_file_name)
x4d.outputPointcloudFiles()

frame_num = x4d.frame_num

# read image and point cloud

print("read image and point cloud")
for i in range(frame_num):
    image = cv2.imread(x4d.image_files[i],1)
    x4d.images.append(image)
    
    point_cloud = open3d.io.read_point_cloud(x4d.point_cloud_files[i])
    x4d.point_clouds.append(point_cloud)
    cv2.imshow("read image",image)
    cv2.waitKey(1)
cv2.destroyAllWindows()

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
pc = open3d.geometry.PointCloud()
vis.add_geometry(pc)
#view_control = vis.get_view_control()
view_control_1 = vis.get_view_control()


param_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()
param_pre_pre_1 = view_control_1.convert_to_pinhole_camera_parameters()

#render_option_1 = vis.get_render_option()
render_option_1 = vis.get_render_option()




pre_time = time.time()*10**6
frame_index = 0
while True:
    

    if time.time()*10**6 - pre_time + x4d.timestamps[frame_index] > x4d.timestamps[frame_index + 1]:
        frame_index += 1
        print("frame ",frame_index)
        pc_new = x4d.point_clouds[frame_index]
       

        vis.remove_geometry(pc)
        
        pc = pc_new        
        vis.add_geometry(pc)
        
        pre_time = time.time()*10**6
        
        if (frame_index + 1 == x4d.frame_num):
            frame_index = 0
        
        # 多分ポインタを取得している
    view_control_1 = vis.get_view_control()
   
    # 前のパラメータを設定
    view_control_1.convert_from_pinhole_camera_parameters(param_pre_1)
    

    render_option_1 = vis.get_render_option()
    
    render_option_1.load_from_json("vis_set_1.json")
   
    #更新
    vis.update_geometry(pc)
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
