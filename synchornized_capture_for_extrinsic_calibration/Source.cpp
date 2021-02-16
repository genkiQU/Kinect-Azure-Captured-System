// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma warning(disable:4996)

#include <k4a/k4a.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2\opencv.hpp>
#include <chrono>
#include <direct.h>
#include "transform_main.h"
#include "calibparam.h"

static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}

static void generate_point_cloud(const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int *point_count)
{
	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
	k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
			point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
			point_cloud_data[i].xyz.z = (float)depth_data[i];
			(*point_count)++;
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

cv::Mat convertColorImageK4a2Cv(k4a_image_t k4a_image) {
	if (k4a_image != NULL)
	{
		// you can check the format with this function
		k4a_image_format_t format = k4a_image_get_format(k4a_image); // K4A_IMAGE_FORMAT_COLOR_BGRA32 

																	  // get raw buffer
		uint8_t* buffer = k4a_image_get_buffer(k4a_image);

		// convert the raw buffer to cv::Mat
		int rows = k4a_image_get_height_pixels(k4a_image);
		int cols = k4a_image_get_width_pixels(k4a_image);
		cv::Mat cvMat(rows, cols, CV_8UC4, (void*)buffer, cv::Mat::AUTO_STEP);

		//std::cout << "rows " << rows << " px" << std::endl;
		//std::cout << "cols " << cols << " px" << std::endl;


		// ...

		k4a_image_release(k4a_image);
		return cvMat;
	}
	else {
		cv::Mat cvMat;
		return cvMat;
	}
}

static void write_point_cloud(const char *file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	// save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

int showTimestamp(k4a_image_t k4a_image) {
	auto device_time_color = k4a_image_get_device_timestamp_usec(k4a_image);
	std::cout << "device timestamp [us] " << device_time_color << " : ";
	auto system_time_color = k4a_image_get_system_timestamp_nsec(k4a_image);
	std::cout << "system timestamp [ns] " << system_time_color << std::endl;

	return device_time_color;
}

bool calibrateMultipleDeviceTimeStamp(std::vector<k4a_device_t> &devices, int master_index) {
	std::cout << "calibrating mutliple device ... " << std::endl;
	std::cout << "master index is " << master_index << std::endl;

	// variaves
	int device_count = devices.size();
	std::vector<k4a_capture_t> captures;
	k4a_image_t depth_image = NULL;
	k4a_image_t color_image = NULL;
	cv::Mat cv_color_image;
	const int32_t TIMEOUT_IN_MS = 1000;

	for (int i = 0; i < device_count; i++) {
		k4a_capture_t capture = NULL;
		captures.push_back(capture);
	}
	
	// capture start
	const int timeout_ms = 10000;
	int* times;
	int* pre_times;
	times = new int[device_count];
	pre_times = new int[device_count];
	for (int i = 0; i < 25; i++) {
		for (int device_t = 0; device_t < device_count; device_t++) {
			switch (k4a_device_get_capture(devices[device_t], &captures[device_t], TIMEOUT_IN_MS))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				printf("Timed out waiting for a capture\n");
				return false;
			case K4A_WAIT_RESULT_FAILED:
				printf("Failed to read a capture\n");
				return false;
			}
		}
		
		for (int device_t = 0; device_t < device_count; device_t++) {
			// Get a depth image

			depth_image = k4a_capture_get_depth_image(captures[device_t]);
			if (depth_image == 0)
			{
				printf("Failed to get depth image from capture\n");
				return false;
			}
			// Get a Color image

			if (i < 30) {
				color_image = k4a_capture_get_color_image(captures[device_t]);
				if (color_image == 0)
				{
					printf("Failed to get color image from capture\n");
					return false;
				}
				cv_color_image = convertColorImageK4a2Cv(color_image);
				if (device_t == master_index) {
					cv::imwrite("test_master.png", cv_color_image);
				}
				else {
					cv::imwrite("test_sub.png", cv_color_image);
				}
			}
			//showTimestamp(color_image);
			pre_times[device_t] = times[device_t];
			times[device_t] = showTimestamp(depth_image);
		}
	}
	std::cout << "calibrating done " << std::endl;
	std::cout << "check each device times" << std::endl;
	for (int i = 0; i < device_count; i++) {
		std::cout << "device " << i << " ; " << times[i] <<" us "<< std::endl;
	}
	int diff = 0;
	std::cout << "total diff times " << diff << std::endl;
	for (int i = 0; i < device_count; i++) {
		diff += abs(times[master_index] - times[i]);
	}
	std::cout << "total diff times " << diff << std::endl;
	delete times;
	delete pre_times;
	if (diff != 0) {
		return false;
	}
	

	return true;
}

bool calibrateDepthAndColorDeviceTimeStamp(k4a_device_t &device) {
	
	std::cout << "calibrate depth and color timestamp" << std::endl;

	k4a_capture_t capture = NULL;
	k4a_image_t depth_image = NULL;
	k4a_image_t color_image = NULL;
	cv::Mat cv_color_image;
	const int32_t TIMEOUT_IN_MS = 1000;

	// capture start
	const int timeout_ms = 10000;
	int depth_time,color_time;
	int total_diff = 0; // depth - color

	int use_image_num = 50;

	for (int i = 0; i < use_image_num; i++) {

		switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
		{
		case K4A_WAIT_RESULT_SUCCEEDED:
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			printf("Timed out waiting for a capture\n");
			return false;
		case K4A_WAIT_RESULT_FAILED:
			printf("Failed to read a capture\n");
			return false;
		}
		

			//Get a depth image
			depth_image = k4a_capture_get_depth_image(capture);
			if (depth_image == 0)
			{
				printf("Failed to get depth image from capture\n");
				return false;
			}
			// Get a Color image
			color_image = k4a_capture_get_color_image(capture);
			if (color_image == 0)
			{
				printf("Failed to get color image from capture\n");
				return false;
			}
				
			//showTimestamp(color_image);
			color_time = showTimestamp(color_image);
			depth_time = showTimestamp(depth_image);
			std::cout << "diff " << (depth_time - color_time) << std::endl;
			total_diff += (depth_time - color_time);
	}
	std::cout << "average diff times is " << (total_diff / use_image_num)<<"us" << std::endl;


	return true;
}

void outputX4D(std::string base_folder, k4a_device_t &device,std::vector<k4a_capture_t> &buffer,k4a_device_configuration_t config,k4a_image_t &xy_table,
	k4a_image_t &point_cloud) {
	// base_folder
	// - color_image
	// - point_cloud
	// - main.x4d
	

	// make dirs
	mkdir(base_folder.c_str());
	std::string image_folder = base_folder + "/image";
	mkdir(image_folder.c_str());
	std::string point_cloud_folder = base_folder + "/point_cloud";
	mkdir(point_cloud_folder.c_str());

	std::cout << "output buffer to " << base_folder << std::endl;

	int buffer_size = buffer.size();
	std::cout << "buffer size is " << buffer_size << std::endl;

	std::vector<int> timestamps;
	std::vector<std::string> point_cloud_files;
	std::vector<std::string> color_image_files;


	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
	{
		printf("Failed to get calibration\n");
		return;
	}
	k4a_transformation_t transformation = k4a_transformation_create(&calibration);

	// output point cloud data and timestampe data
	k4a_image_t color_image = NULL;
	k4a_image_t depth_image = NULL;
	//k4a_image_t xy_table =NULL;
	//k4a_image_t point_cloud = NULL;
	int point_count = 0;
	for (int frame_t = 0; frame_t < buffer_size; frame_t++) {

		k4a_capture_t capture = buffer[frame_t];
		depth_image = k4a_capture_get_depth_image(capture);
		if (depth_image == 0)
		{
			printf("Failed to get depth image from capture\n");
			return;
		}
		// Get a Color image
		color_image = k4a_capture_get_color_image(capture);
		if (color_image == 0)
		{
			printf("Failed to get color image from capture\n");
			return;
		}
		//generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);
		std::string point_filename = point_cloud_folder + "/ply" + std::to_string(frame_t) + ".ply";
		//write_point_cloud(point_filename.c_str(), point_clouds[i], point_count);
		point_cloud_color_to_depth(transformation, depth_image, color_image, point_filename);
		//point_cloud_depth_to_color(transformation, depth_image, color_image, point_filename);

		//
		cv::Mat cv_colorImage = convertColorImageK4a2Cv(color_image);
		std::string color_filename = image_folder + "/image" + std::to_string(frame_t) + ".png";
		cv::imwrite(color_filename, cv_colorImage);

		int timestamp = k4a_image_get_device_timestamp_usec(depth_image);

		point_filename = "point_cloud/ply" + std::to_string(frame_t) + ".ply";
		color_filename = "image/image" + std::to_string(frame_t) + ".png";

		// push_back
		point_cloud_files.push_back(point_filename);
		color_image_files.push_back(color_filename);
		timestamps.push_back(timestamp);
	}

	// output x4d
	std::string x4d_filename = base_folder + "/x4d.x4d";
	std::ofstream ofs(x4d_filename);
	ofs << "Frame_num " << buffer_size << std::endl;
	ofs << "Data{" << std::endl;
	ofs << "ply" << std::endl;
	ofs << "timestamp" << std::endl;
	ofs << "color_image" << std::endl;
	ofs << "}" << std::endl;
	
	// ply files
	for (int i = 0; i < buffer_size; i++) {
		ofs << point_cloud_files[i] << std::endl;
	}
	for (int i = 0; i < buffer_size; i++) {
		ofs << std::to_string(timestamps[i]) << std::endl;
	}
	for (int i = 0; i < buffer_size; i++) {
		ofs << color_image_files[i] << std::endl;
	}
	ofs.close();

	return;
}

void timeStampComp(std::vector<k4a_capture_t> &capture0, std::vector<k4a_capture_t> &capture1, std::string outputFile) {
	int frame_num = capture0.size();
	std::ofstream ofs(outputFile);

	for (int i = 0; i < frame_num; i++) {
		k4a_capture_t cap0 = NULL;
		k4a_capture_t cap1 = NULL;

		cap0 = capture0[i];
		cap1 = capture1[i];
		
		int time0;
		int time1;


		k4a_image_t depth_image0 = NULL;
		k4a_image_t depth_image1 = NULL;
		depth_image0 = k4a_capture_get_depth_image(cap0);
		if (depth_image0 == 0)
		{
			printf("Failed to get depth image from capture\n");
			return;
		}
		// Get a Color image
		depth_image1 = k4a_capture_get_depth_image(cap1);
		if (depth_image1 == 0)
		{
			printf("Failed to get color image from capture\n");
			return;
		}

		time0 = k4a_image_get_device_timestamp_usec(depth_image0);
		time1 = k4a_image_get_device_timestamp_usec(depth_image1);
		int diff = time0 - time1;

		ofs << time0 << "," << time1 << "," << diff << std::endl;
	}
	ofs.close();
}

int main(int argc, char **argv)
{
	int returnCode = 1;
	std::vector<k4a_device_t> devices;
	const int32_t TIMEOUT_IN_MS = 1000;
	k4a_capture_t capture = NULL;
	std::vector<k4a_capture_t> captures;
	std::string file_name;
	uint32_t device_count = 0;
	k4a_device_configuration_t config_mas = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_device_configuration_t config_sub = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_device_configuration_t config_stand = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_image_t depth_image = NULL;
	k4a_image_t color_image = NULL;
	cv::Mat cv_color_image;
	std::vector<k4a_image_t> xy_tables;
	std::vector<k4a_image_t> point_clouds;
	int point_count = 0;
	std::vector<k4a_device_configuration_t> configs;
	std::vector<k4a_transformation_t> transformations;

	std::cout << "argv[1] : wait time [ms]" << std::endl;
	std::cout << "argv[2] : capture image number " << std::endl;

	int wait_time = atoi(argv[1]);
	int capture_image_num = atoi(argv[2]);

	// output file
	//std::vector<std::string> x4d_filenames;
	//std::vector<std::vector<std::string>> point_cloud_file_lists;
	//std::vector<std::vector<int>> time_stamp_lists;

	// capture_t buffers
	std::vector<std::vector<k4a_capture_t>> buffers;


	file_name = argv[1];

	// 接続デバイス数の取得
	device_count = k4a_device_get_installed_count();
	std::cout << "device number is " << device_count << std::endl;
	if (device_count == 0)
	{
		printf("No K4A devices found\n");
		return 0;
	}
	// デバイスの初期化
	for (int i = 0; i < device_count; i++) {
		k4a_device_t device = NULL;
		devices.push_back(device);
		k4a_capture_t capture = NULL;
		captures.push_back(capture);
		k4a_image_t xy_table = NULL;
		k4a_image_t point_cloud = NULL;
		xy_tables.push_back(xy_table);
		point_clouds.push_back(point_cloud);
	}
	buffers.resize(device_count);
	transformations.resize(device_count);

	// デバイスのオープン
	for (int i = 0; i < device_count; i++) {
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &devices[i]))
		{
			printf("Failed to open device %d \n",&i);
			goto Exit;
		}
	}

	//　デバイス情報の取得
	for (int i = 0; i < device_count; i++) {
		std::cout << "device " << i << " info " << std::endl;

		size_t serial_size = 0;
		k4a_device_get_serialnum(devices[i], NULL, &serial_size);

		// Allocate memory for the serial, then acquire it
		char *serial = (char*)(malloc(serial_size));
		k4a_device_get_serialnum(devices[i], serial, &serial_size);
		printf("Opened device: %s\n", serial);
		free(serial);
		std::cout << std::endl;
	}
	// detect master camera
	int master_index = 0;
	if (device_count == 0) {
		std::cout << "No device is detected ! " << std::endl;
	}
	else if (device_count == 1) {
		std::cout << " 1 device is detected ! " << std::endl;
		master_index = 0;
	}
	else {
		std::cout << " multiple devices are detected ! " << std::endl;
		std::cout << "search the master device ..." << std::endl;
		for (int i = 0; i < device_count; i++) {
			bool syncOutJack,syncInJack;
			k4a_device_get_sync_jack(devices[i], &syncInJack, &syncOutJack);
			//std::cout << "device " << i << " "<< syncInJack <<" "<< syncOutJack;
			if (syncOutJack) master_index = i;
		}
		std::cout << "master device is " << master_index << std::endl;
	}

	// master camera config
	config_mas = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config_mas.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config_mas.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config_mas.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config_mas.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config_mas.synchronized_images_only = true;
	config_mas.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;

	// subcamera config
	config_sub = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config_sub.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config_sub.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config_sub.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config_sub.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config_sub.synchronized_images_only = true;
	config_sub.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	config_sub.subordinate_delay_off_master_usec = 0;

	// subcamera config
	config_stand = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config_stand.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config_stand.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config_stand.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config_stand.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config_stand.synchronized_images_only = true;
	config_stand.subordinate_delay_off_master_usec = 0;


	// setting device
	std::cout << "setting device .. " << std::endl;
	for (int i = 0; i < device_count; i++) {

		// extract calibration data
		k4a_calibration_t calibration;
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(devices[i], config_stand.depth_mode, config_stand.color_resolution, &calibration))
		{
			printf("Failed to get calibration\n");
			goto Exit;
		}
		
		// image create
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			calibration.depth_camera_calibration.resolution_width,
			calibration.depth_camera_calibration.resolution_height,
			calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
			&xy_tables[i]);
		create_xy_table(&calibration, xy_tables[i]);

		// image create
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			calibration.depth_camera_calibration.resolution_width,
			calibration.depth_camera_calibration.resolution_height,
			calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
			&point_clouds[i]);

		k4a_transformation_t transformation;
		transformation = k4a_transformation_create(&calibration);
		transformations.push_back(transformation);
		
	}
	std::cout << "setting device done" << std::endl;


	std::cout << "start device " << std::endl;
	if (device_count > 1) {
		// setting sub-nodes
		for (int i = 0; i < device_count; i++) {
			if (i == master_index)continue;
			if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(devices[i], &config_sub))
			{
				printf("Failed to start cameras\n");
				goto Exit;
			}
		}
		// setting master-node
		if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(devices[master_index], &config_mas))
		{
			printf("Failed to start cameras\n");
			goto Exit;
		}
		std::cout << "all device is started" << std::endl;
	}
	else {
		if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(devices[0], &config_stand))
		{
			printf("Failed to start cameras\n");
			goto Exit;
		}
	}

	

	
	// exposure seeting
	std::cout << "exposure setting .." << std::endl;
	for (int i = 0; i < device_count; i++) {
		std::chrono::microseconds exposure_time(30000);
		int exposure_time_int = 30000;
		 //k4a_set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, exposure_time.count());
		k4a_device_set_color_control(devices[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, exposure_time_int);
		//k4a_device_set_color_control(devices[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, exposure_time_int);
		//K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY 
		int32_t value = 0;
		k4a_color_control_mode_t mode;
		//device.get_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, &mode, &value);
		k4a_device_get_color_control(devices[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, &mode, &value);
		/* "Manual 500" is displayed on console */
		std::cout << std::to_string(mode) << " " << value << "\n";
	}
	std::cout << "exposure setting done" << std::endl;


	// multiple device timestamp calibrating
	if (device_count > 1) {
		// multiple device timestamp calibrating
		while (true) {
			if (calibrateMultipleDeviceTimeStamp(devices, master_index) == true) {
				break;
			}
		}
	}
	//calibrateMultipleDeviceTimeStamp(devices, master_index);
	// depth image and color image calibration
	//for (int i = 0; i < device_count; i++) {
	//	calibrateDepthAndColorDeviceTimeStamp(devices[i]);
	//}

	// multiple image captured
	const int timeout_ms = 10000;
	int frame_count = 0;
	int max_frame = capture_image_num;
	for (int i = 0; i < max_frame; i++) {
		// Get a capture
		if (frame_count % 10 == 0) {
			std::cout<<"capture frame " << frame_count << std::endl;
		}

		for (int device_t = 0; device_t < device_count; device_t++) {
			switch (k4a_device_get_capture(devices[device_t], &captures[device_t], TIMEOUT_IN_MS))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				printf("Timed out waiting for a capture\n");
				goto Exit;
			case K4A_WAIT_RESULT_FAILED:
				printf("Failed to read a capture\n");
				goto Exit;
			}
		}
		for (int device_t = 0; device_t < device_count; device_t++) {
			buffers[device_t].push_back(captures[device_t]);

			// Get a depth image
			std::cout << "device " << device_t<<" ";
			depth_image = k4a_capture_get_depth_image(captures[device_t]);
			if (depth_image == 0)
			{
				printf("Failed to get depth image from capture\n");
				goto Exit;
			}
			// Get a Color image
			color_image = k4a_capture_get_color_image(captures[device_t]);
			if (color_image == 0)
			{
				printf("Failed to get color image from capture\n");
				goto Exit;
			}


			//showTimestamp(color_image);
			showTimestamp(depth_image);
			
			//generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);
			//write_point_cloud(file_name.c_str(), point_cloud, point_count);
			cv::Mat colorImage;
			colorImage = convertColorImageK4a2Cv(color_image);
			std::string window_name = "capture " + std::to_string(device_t);
			cv::imshow(window_name, colorImage);

		}
		cv::waitKey(wait_time);
		// one frame end
		frame_count++;
		std::cout << std::endl;
	}
	cv::destroyAllWindows();

	
	// analysis buffer data
	std::cout << "buffer data analysis ... " << std::endl;
	for (int i = 0; i < device_count; i++) {
		std::cout << "device " << i << std::endl;
		auto buffer = buffers[i];
		int buffer_size = buffer.size();
		std::cout << "buffer size is " << buffer_size << std::endl;

		// buffer time stamp analysis
		if (0) {
			int pre_time, cur_time;
			for (int frame_t = 0; frame_t < buffer_size; frame_t++) {
				k4a_capture_t capture = buffer[frame_t];
				depth_image = k4a_capture_get_depth_image(capture);
				if (depth_image == 0)
				{
					printf("Failed to get depth image from capture\n");
					goto Exit;
				}
				// Get a Color image
				color_image = k4a_capture_get_color_image(capture);
				if (color_image == 0)
				{
					printf("Failed to get color image from capture\n");
					goto Exit;
				}
				std::cout << "frame " << frame_t<<" ";
				if (frame_t == 0) {
					cur_time = showTimestamp(depth_image);
				}
				else {
					pre_time = cur_time;
					cur_time = showTimestamp(depth_image);
					std::cout << "time per frame " << (cur_time - pre_time) << "us " << std::endl;
				}
			}
		}
	
		k4a_calibration_t calibration;
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(devices[i], config_stand.depth_mode, config_stand.color_resolution, &calibration))
		{
			printf("Failed to get calibration\n");
			goto Exit;
		}
		std::string base_folder = "output_" + std::to_string(i);
		outputX4D(base_folder, devices[i], buffer,config_stand,xy_tables[i],point_clouds[i]);

		//k4a_transformation_t transformation = k4a_transformation_create(&calibration);

		// output point cloud data and timestampe data
		/*for (int frame_t = 0; frame_t < buffer_size; frame_t++) {
			
			k4a_capture_t capture = buffer[frame_t];
			depth_image = k4a_capture_get_depth_image(capture);
			if (depth_image == 0)
			{
				printf("Failed to get depth image from capture\n");
				goto Exit;
			}
			// Get a Color image
			color_image = k4a_capture_get_color_image(capture);
			if (color_image == 0)
			{
				printf("Failed to get color image from capture\n");
				goto Exit;
			}
			generate_point_cloud(depth_image, xy_tables[i], point_clouds[i], &point_count);
			std::string filename = "output/point" + std::to_string(frame_t)+".ply";
			//write_point_cloud(filename.c_str(), point_clouds[i], point_count);
			point_cloud_color_to_depth(transformation, depth_image, color_image, filename);
		}*/
	}

	timeStampComp(buffers[0], buffers[1],"timestamp.csv");
	
	print_calibration(devices,config_stand);

	// release all image
	k4a_image_release(depth_image);
	for (int i = 0; i < device_count; i++) {
		std::string base_folder = "output_" + std::to_string(i)+"/params";
		write_calibration(devices[i], config_stand, base_folder);
		k4a_capture_release(captures[i]);
		k4a_image_release(xy_tables[i]);
		k4a_image_release(point_clouds[i]);
	}
	returnCode = 0;
Exit:
	for (int i = 0; i < device_count; i++) {
		if (devices[i] != NULL)
		{
			k4a_device_close(devices[i]);
		}
	}
	
	return returnCode;
}