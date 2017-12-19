#pragma once
#include "stdafx.h"

#ifndef HKKIM_FUNCTION_H
#define HKKIM_FUNCTION_H


//.. Dataset Option
#define ROAD_CATEGORY 1 

// Testing Dataset
#if (ROAD_CATEGORY == 1)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/testing/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "um"
#define FILE_START 0
#define FILE_END 95
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

#if (ROAD_CATEGORY == 2)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/testing/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "umm"
#define FILE_START 0
#define FILE_END 93
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

#if (ROAD_CATEGORY == 3)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/testing/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "uu"
#define FILE_START 0
#define FILE_END 99
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

// Training Dataset
#if (ROAD_CATEGORY == 4)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/training/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/training/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "um"
#define FILE_START 0
#define FILE_END 94
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

#if (ROAD_CATEGORY == 5)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/training/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/training/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "umm"
#define FILE_START 0
#define FILE_END 95
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

#if (ROAD_CATEGORY == 6)
#define KITTI_I_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/training/image_2"
#define KITTI_L_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road_velodyne/training/velodyne"
#define KITTI_Calib_PATH "D:/phD_Candidates/VC2015/RoadLane_Detection_Evaluation/data_road/data_road/testing/calib/um_000000.txt"
#define FILE_NAME "uu"
#define FILE_START 0
#define FILE_END 97
#define IMAGE_FILE_FORMAT "png"
#define LIDAR_FILE_FORMAT "bin"
#endif

struct PointCloud {
	float px, py, pz, pr;
	PointCloud() {
		px = py = pz = pr = NULL;
	}
	PointCloud(float x, float y, float z, float r) {
		px = x;
		py = y;
		pz = z;
		pr = r;
	}
};

struct RangeImage {
	Mat floatRangeImage, GrayReflectionImage, ColorReflectionImage;
	
	RangeImage(Size imageSize) 
	{
		Mat t1(imageSize, CV_32FC1, Scalar(0));
		Mat t2(imageSize, CV_8UC1, Scalar(0));
		Mat t3(imageSize, CV_8UC3, Scalar(0));

		floatRangeImage = t1;
		GrayReflectionImage = t2;
		ColorReflectionImage = t3;
	}
};

//.. Global Variable

//.. Functions
Mat KITTI_ReadColorImage(string openImagePath);
void KITTI_ReadVelodyne(string openLidarPath, Size imageSize, Mat projectionMatrix, RangeImage outputDepth);
void floatDepthImage2GrayImage(cv::Mat floatDepth, cv::Mat GrayImage);
bool readProjectionMatrix(string openCalibFile, cv::Mat& projectionMatrix);

#endif