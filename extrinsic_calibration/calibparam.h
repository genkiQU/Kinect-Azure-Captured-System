#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <k4a/k4a.h>
using namespace std;

static string get_serial(k4a_device_t device)
{
	size_t serial_number_length = 0;

	if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
	{
		cout << "Failed to get serial number length" << endl;
		k4a_device_close(device);
		exit(-1);
	}

	char *serial_number = new (std::nothrow) char[serial_number_length];
	if (serial_number == NULL)
	{
		cout << "Failed to allocate memory for serial number (" << serial_number_length << " bytes)" << endl;
		k4a_device_close(device);
		exit(-1);
	}

	if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
	{
		cout << "Failed to get serial number" << endl;
		delete[] serial_number;
		serial_number = NULL;
		k4a_device_close(device);
		exit(-1);
	}

	string s(serial_number);
	delete[] serial_number;
	serial_number = NULL;
	return s;
}

void write_calibration(k4a_device_t& device,k4a_device_configuration_t& deviceConfig ,std::string folder_name) {
	// output parameter to filename
	k4a_calibration_t calibration;


	// get calibration
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &calibration))
	{
		cout << "Failed to get calibration" << endl;
		exit(-1);
	}

	auto calib = calibration.depth_camera_calibration;

	cout << "\n===== Device " << get_serial(device) << " =====" << std::endl;
	std::cout << " output to " << folder_name << std::endl;
	mkdir(folder_name.c_str());

	cv::Mat depth_k = cv::Mat::zeros(3,3, CV_32F);
	depth_k.at<float>(0, 0) = calib.intrinsics.parameters.param.fx;
	depth_k.at<float>(1, 1) = calib.intrinsics.parameters.param.fy;
	depth_k.at<float>(0, 2) = calib.intrinsics.parameters.param.cx;
	depth_k.at<float>(1, 2) = calib.intrinsics.parameters.param.cy;
	depth_k.at<float>(2, 2) = 1.0;
	cv::Mat depth_d = cv::Mat::zeros(8, 1, CV_32F);
	depth_d.at<float>(0, 0) = calib.intrinsics.parameters.param.k1;
	depth_d.at<float>(1, 0) = calib.intrinsics.parameters.param.k2;
	depth_d.at<float>(2, 0) = calib.intrinsics.parameters.param.p1;
	depth_d.at<float>(3, 0) = calib.intrinsics.parameters.param.p2;
	depth_d.at<float>(4, 0) = calib.intrinsics.parameters.param.k3;
	depth_d.at<float>(5, 0) = calib.intrinsics.parameters.param.k4;
	depth_d.at<float>(6, 0) = calib.intrinsics.parameters.param.k3;
	depth_d.at<float>(7, 0) = calib.intrinsics.parameters.param.k4;
	cv::Vec2i depth_size;
	depth_size(0) = calib.resolution_width;
	depth_size(1) = calib.resolution_height;


	calib = calibration.color_camera_calibration;
	cv::Mat color_k = cv::Mat::zeros(3, 3, CV_32F);
	color_k.at<float>(0, 0) = calib.intrinsics.parameters.param.fx;
	color_k.at<float>(1, 1) = calib.intrinsics.parameters.param.fy;
	color_k.at<float>(0, 2) = calib.intrinsics.parameters.param.cx;
	color_k.at<float>(1, 2) = calib.intrinsics.parameters.param.cy;
	color_k.at<float>(2, 2) = 1.0;
	cv::Mat color_d = cv::Mat::zeros(8, 1, CV_32F);
	color_d.at<float>(0, 0) = calib.intrinsics.parameters.param.k1;
	color_d.at<float>(1, 0) = calib.intrinsics.parameters.param.k2;
	color_d.at<float>(2, 0) = calib.intrinsics.parameters.param.p1;
	color_d.at<float>(3, 0) = calib.intrinsics.parameters.param.p2;
	color_d.at<float>(4, 0) = calib.intrinsics.parameters.param.k3;
	color_d.at<float>(5, 0) = calib.intrinsics.parameters.param.k4;
	color_d.at<float>(6, 0) = calib.intrinsics.parameters.param.k3;
	color_d.at<float>(7, 0) = calib.intrinsics.parameters.param.k4;
	cv::Vec2i color_size;
	color_size(0) = calib.resolution_width;
	color_size(1) = calib.resolution_height;

	// wirte parameters to folder_name
	std::string filename;
	filename = folder_name + "/depth_k.txt";
	std::ofstream ofs;
	ofs.open(filename);
	for (int i = 0; i < 3; i++) {
		ofs << depth_k.at<float>(i, 0) << " " << depth_k.at<float>(i, 1) << " " << depth_k.at<float>(i, 2) << std::endl;
	}
	ofs.close();

	filename = folder_name + "/depth_d.txt";
	ofs.open(filename);
	for (int i = 0; i < 8; i++) {
		ofs << depth_d.at<float>(i, 0) << std::endl;
	}
	ofs.close();

	filename = folder_name + "/depth_size.txt";
	ofs.open(filename);
	ofs << depth_size(0) << " " << depth_size(1) << std::endl;
	ofs.close();

	filename = folder_name + "/color_k.txt";
	ofs.open(filename);
	for (int i = 0; i < 3; i++) {
		ofs << color_k.at<float>(i, 0) << " " << color_k.at<float>(i, 1) << " " << color_k.at<float>(i, 2) << std::endl;
	}
	ofs.close();

	
	filename = folder_name + "/color_d.txt";
	ofs.open(filename);
	for (int i = 0; i < 8; i++) {
		ofs << color_d.at<float>(i, 0) << std::endl;
	}
	ofs.close();

	
	filename = folder_name + "/color_size.txt";
	ofs.open(filename);
	ofs << color_size(0) << " " << color_size(1) << std::endl;
	ofs.close();
	
}

static void print_calibration(std::vector<k4a_device_t>& devices, k4a_device_configuration_t& deviceConfig)
{
	uint32_t device_count = devices.size();
	

	for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
	{
		k4a_device_t& device = devices[deviceIndex];

		k4a_calibration_t calibration;


		// get calibration
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &calibration))
		{
			cout << "Failed to get calibration" << endl;
			exit(-1);
		}

		auto calib = calibration.depth_camera_calibration;

		cout << "\n===== Device " << (int)deviceIndex << ": " << get_serial(device) << " =====\n";
		cout << "depth " << std::endl;
		cout << "resolution width: " << calib.resolution_width << endl;
		cout << "resolution height: " << calib.resolution_height << endl;
		cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
		cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
		cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
		cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
		cout << "radial distortion coefficients:" << endl;
		cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
		cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
		cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
		cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
		cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
		cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
		cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
		cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
		cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
		cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
		cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;

		calib = calibration.color_camera_calibration;
		cout << "\n===== Device " << (int)deviceIndex << ": " << get_serial(device) << " =====\n";
		cout << "depth " << std::endl;
		cout << "resolution width: " << calib.resolution_width << endl;
		cout << "resolution height: " << calib.resolution_height << endl;
		cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
		cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
		cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
		cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
		cout << "radial distortion coefficients:" << endl;
		cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
		cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
		cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
		cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
		cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
		cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
		cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
		cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
		cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
		cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
		cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
	}
}

static void calibration_blob(uint8_t deviceIndex = 0, string filename = "calibration.json")
{
	k4a_device_t device = NULL;

	if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
	{
		cout << deviceIndex << ": Failed to open device" << endl;
		exit(-1);
	}

	size_t calibration_size = 0;
	k4a_buffer_result_t buffer_result = k4a_device_get_raw_calibration(device, NULL, &calibration_size);
	if (buffer_result == K4A_BUFFER_RESULT_TOO_SMALL)
	{
		vector<uint8_t> calibration_buffer = vector<uint8_t>(calibration_size);
		buffer_result = k4a_device_get_raw_calibration(device, calibration_buffer.data(), &calibration_size);
		if (buffer_result == K4A_BUFFER_RESULT_SUCCEEDED)
		{
			ofstream file(filename, ofstream::binary);
			file.write(reinterpret_cast<const char *>(&calibration_buffer[0]), (long)calibration_size);
			file.close();
			cout << "Calibration blob for device " << (int)deviceIndex << " (serial no. " << get_serial(device)
				<< ") is saved to " << filename << endl;
		}
		else
		{
			cout << "Failed to get calibration blob" << endl;
			exit(-1);
		}
	}
	else
	{
		cout << "Failed to get calibration blob size" << endl;
		exit(-1);
	}
}