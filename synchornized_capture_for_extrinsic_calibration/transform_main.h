#pragma once
#include "transformation_helpers.h"
#include <string>

bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image,
	std::string file_name);
bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image,
	std::string file_name);
