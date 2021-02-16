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


bool readCamMat(std::string filename, cv::Mat& cam_k) {
	std::ifstream ifs(filename);

	if (ifs.fail()) {
		std::cout << "cannnot open " << filename << std::endl;
		return false;
	}
	ifs.close();

	FILE *fp;
	fp = fopen(filename.c_str(), "r");
	float a, b, c;
	for (int i = 0; i < 3; i++) {
		fscanf(fp, "%f %f %f", &a, &b, &c);
		cam_k.at<float>(i, 0) = a;
		cam_k.at<float>(i, 1) = b;
		cam_k.at<float>(i, 2) = c;
	}
	fclose(fp);

	std::cout << "reaading from " << filename << std::endl;
	std::cout << cam_k << std::endl;

	return true;
}

bool readDist(std::string filename, std::vector<float>& dist) {
	std::ifstream ifs(filename);

	if (ifs.fail()) {
		std::cout << "cannnot open " << filename << std::endl;
		return false;
	}
	ifs.close();

	FILE *fp;
	fp = fopen(filename.c_str(), "r");

	for (int i = 0; i < 8; i++) {
		float a;
		fscanf(fp, "%f", &a);
		dist.push_back(a);
	}
	fclose(fp);

	std::cout << "reaading from " << filename << std::endl;
	
	for (int i = 0; i < 8; i++) {
		std::cout << " " << dist[i];
	}
	std::cout << std::endl;

	return true;
}

std::string sprintf_string(std::string format, int index) {
	char tmp[256];

	sprintf(tmp, format.c_str(), index);
	
	return std::string{ tmp };
}

class Marker {
public:
	int id;
	float u, v;
	float x, y, z;

	Marker(int id_,float u_, float v_,float x_,float y_,float z_){
		id = id_;
		u = u_;
		v = v_;
		x = x_;
		y = y_;
		z = z_;
	}
};

std::vector<Marker> readingMarkers(std::string cam2d_file, std::string cam3d_file) {

	std::vector<Marker> markers;

	std::cout << "reading marker files" << std::endl;
	std::cout << "cam2d from " << cam2d_file << std::endl;
	std::cout << "cam3d from " << cam3d_file << std::endl;

	// count marker number
	int marker_num = 0;
	{
		marker_num = 0;
		std::ifstream ifs(cam2d_file);
		std::string tmp;
		while (std::getline(ifs, tmp))
			marker_num++;

		ifs.close();
	}
	std::cout << "marker number is " << marker_num << std::endl;

	// reading cam2dfile and cam3dfile simulataneous
	FILE *fp_2d = fopen(cam2d_file.c_str(), "r");
	FILE *fp_3d = fopen(cam3d_file.c_str(), "r");

	for (int i = 0; i<marker_num; i++) {
		int marker_id;
		float x_, y_, z_;
		float u_, v_;
		fscanf(fp_2d, "%d %f %f", &marker_id, &u_, &v_);
		fscanf(fp_3d, "%d %f %f %f", &marker_id, &x_, &y_, &z_);
		Marker marker(marker_id, u_, v_,x_,y_,z_);
		markers.push_back(marker);
	}
	fclose(fp_2d);
	fclose(fp_3d);

	std::cout << "reading done" << std::endl;

	return markers;
}


int main(int argc, char* argv[]) {

	std::cout << "device 0 folder" << std::endl;
	std::cout << "device 1 folder " << std::endl;
	std::cout << "start index" << std::endl;
	std::cout << "end index" << std::endl;
	
	// 
	std::string device0_folder{ argv[1] };
	std::string device1_folder{ argv[2] };
	int start_index = atoi(argv[3]);
	int end_index = atoi(argv[4]);

	//
	std::string image_file_format = "/image/image%d.png";
	std::string cam2D_file_format = "/ARCalib_Project/cam/point/cam2D%03d.txt";
	std::string cam3D_file_format = "/ARCalib_Project/cam/point/cam3D%03d.txt";
	//std::string cam_k_file = "/params/color_k.txt";
	//std::string cam_d_file = "/params/color_d.txt";
	std::string cam_k_file = "/ARCalib_Project/cam/param/camMat.txt";
	std::string cam_d_file = "/ARCalib_Project/cam/param/dist.txt";

	// reading cam mat
	cv::Mat device0_k = cv::Mat::zeros(3, 3, CV_32F);
	cv::Mat device1_k = cv::Mat::zeros(3, 3, CV_32F);
	

	if (!readCamMat(device0_folder + cam_k_file,device0_k)) {
		std::cout << "cannnot open " << device0_folder + cam_k_file << std::endl;
		return 0;
	}
	if (!readCamMat(device1_folder + cam_k_file, device1_k)) {
		std::cout << "cannnot open " << device1_folder + cam_k_file << std::endl;
		return 0;
	}

	// reading dist coef
	std::vector<float> device0_d;
	std::vector<float> device1_d;
	//cv::Mat device0_d_mat,device1_d_mat;
	if (!readDist(device0_folder + cam_d_file, device0_d)) {
		std::cout << "cannnot open " << device0_folder + cam_d_file << std::endl;
		return 0;
	}
	if (!readDist(device1_folder + cam_d_file, device1_d)) {
		std::cout << "cannnot open " << device1_folder + cam_d_file << std::endl;
		return 0;
	}
	cv::Mat device0_d_mat(device0_d.size(),1,CV_32F, device0_d.data());
	cv::Mat device1_d_mat(device1_d.size(), 1, CV_32F, device1_d.data());


	// reading image and cam2d and cam3
	std::vector<cv::Mat> images;
	std::vector<std::vector<cv::Vec2f>> cam2Ds;
	std::vector<std::vector<cv::Vec3f>> cam3Ds;

	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints0;
	std::vector<std::vector<cv::Point2f>> imagePoints1;
	std::vector<int> valid_image_id;

	int height, width;

	
	for (int i = start_index; i < end_index; i++) {
		std::vector<std::string> file_list;
		// 0 ,1 image file
		// 2, 3 cam2d
		// 4, 5 cam3d
		// check exists

		// device 0
		std::string image_file_0, image_file_1;
		std::string cam2d_file_0, cam2d_file_1;
		std::string cam3d_file_0, cam3d_file_1;

		image_file_0 = sprintf_string(device0_folder + "/" + image_file_format, i);
		image_file_1 = sprintf_string(device1_folder + "/" + image_file_format, i);


		cam2d_file_0 = sprintf_string(device0_folder + "/" + cam2D_file_format, i);
		cam2d_file_1 = sprintf_string(device1_folder + "/" + cam2D_file_format, i);

		cam3d_file_0 = sprintf_string(device0_folder + "/" + cam3D_file_format, i);
		cam3d_file_1 = sprintf_string(device1_folder + "/" + cam3D_file_format, i);


		

		// check all file exsits or not
		{
			std::ifstream ifs;
			ifs.open(image_file_0);
			if (!ifs.good()) {
				std::cout << "image file 0 not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();

			ifs.open(image_file_1);
			if (!ifs.good()) {
				std::cout << "image file  1not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();

			ifs.open(cam2d_file_0);
			if (!ifs.good()) {
				std::cout << "cam2d file 0 not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();

			ifs.open(cam2d_file_1);
			if (!ifs.good()) {
				std::cout << "cam2d file 1 not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();

			ifs.open(cam3d_file_0);
			if (!ifs.good()) {
				std::cout << "cam3d file 0 not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();

			ifs.open(cam3d_file_1);
			if (!ifs.good()) {
				std::cout << "cam3d flle 1 not good " << std::endl;
				ifs.close();
				continue;
			}
			ifs.close();
		}
		std::cout << image_file_0 << " " << cam2d_file_0 << " " << cam3d_file_0 << std::endl;
		std::cout << image_file_1 << " " << cam2d_file_1 << " " << cam3d_file_1 << std::endl;


		// reading
		cv::Mat image0 = cv::imread(image_file_0, 1);
		cv::Mat image1 = cv::imread(image_file_1, 1);
		height = image0.rows;
		width = image0.cols;

		std::vector<Marker> markers_0, markers_1;
		markers_0 = readingMarkers(cam2d_file_0, cam3d_file_0);
		markers_1 = readingMarkers(cam2d_file_1, cam3d_file_1);

		int marker_0_num = markers_0.size();
		int marker_1_num = markers_1.size();

		std::vector<std::pair<Marker, Marker>> marker_pairs;
		for (int ii = 0; ii < marker_0_num; ii++) {
			for (int jj = 0; jj < marker_1_num; jj++) {
				Marker marker0 = markers_0[ii];
				Marker marker1 = markers_1[jj];
				//std::cout << marker0.id << " " << marker1.id << std::endl;
				if (marker0.id == marker1.id) {
					std::pair<Marker, Marker> marker_pair(marker0, marker1);
					marker_pairs.push_back(marker_pair);
				}
			}
		}

		// show  marker pair
		bool debug = false;
		if (debug) {
			std::cout << "pair result " << std::endl;
			int pair_num = marker_pairs.size();
			for (int ii = 0; ii < pair_num; ii++) {
				auto& pair = marker_pairs[ii];
				auto& marker0 = pair.first;
				auto& marker1 = pair.second;

				std::cout << marker0.id << " " << marker1.id << std::endl;
			}
		}

		// create object list
		int pair_num = marker_pairs.size();
		std::vector<cv::Point3f> objects;
		std::vector<cv::Point2f> points0;
		std::vector<cv::Point2f> points1;

		for (int ii = 0; ii < pair_num; ii++) {
			auto& marker0 = marker_pairs[ii].first;
			auto& marker1 = marker_pairs[ii].second;

			cv::Point3f object;
			object.x = marker0.x;
			object.y = marker0.y;
			object.z = marker0.z;
			//std::cout << marker0.x << " " << marker0.y << std::endl;
			//std::cout << marker1.x << " " << marker1.y << std::endl;
			//std::cout << std::endl;
			if (marker0.x != marker1.x)continue;
			if (marker1.y != marker0.y)continue;
			cv::Point2f point0, point1;
			point0.x = marker0.u;
			point0.y = marker0.v;

			point1.x = marker1.u;
			point1.y = marker1.v;

			std::cout << object << std::endl;
			std::cout << point0 << std::endl;
			std::cout << point1 << std::endl;
			std::cout << std::endl;

			objects.push_back(object);
			points0.push_back(point0);
			points1.push_back(point1);
		}
		if (objects.size() == 0)continue;
		std::cout << "pair number is " << objects.size() << std::endl;
		objectPoints.push_back(objects);
		imagePoints0.push_back(points0);
		imagePoints1.push_back(points1);
		valid_image_id.push_back(i);
	}

	// valid image id list
	std::cout << " valid image pair " << valid_image_id.size() << std::endl;

	// calibrate
	cv::Mat R;
	cv::Mat t;
	cv::Mat E;
	cv::Mat F;
	cv::TermCriteria criteria{ 10000, 10000, 0.0001 };
	double error = cv::stereoCalibrate(objectPoints,
		imagePoints0,
		imagePoints1,
		device0_k,
		device0_d,
		device1_k,
		device1_d,
		cv::Size(width, height),
		R, // output
		t, // output
		E,
		F ,
		cv::CALIB_FIX_INTRINSIC | cv::CALIB_RATIONAL_MODEL,
		criteria);
	std::cout << "Finished calibrating!\n";
	std::cout << "Got error of " << error << "\n";

	std::cout << R << " \n" << t << std::endl;

	// output to output 1 file
	R.convertTo(R, CV_32F, 1.0);
	t.convertTo(t, CV_32F, 1.0);

	mkdir((device1_folder + "/extrinsic").c_str());// , 0777);
	std::ofstream ofs; 
	ofs.open(device1_folder + "/extrinsic/rmat.txt");
	for (int i = 0; i < 3; i++) {
		ofs << R.at<float>(i, 0) << " " << R.at<float>(i, 1) << " " << R.at<float>(i, 2) << std::endl;
	}
	ofs.close();
	ofs.open(device1_folder + "/extrinsic/tmat.txt");
	ofs << t.at<float>(0, 0) << " " << t.at<float>(1, 0) << " " << t.at<float>(2.0) << std::endl;

	ofs.close();
}
