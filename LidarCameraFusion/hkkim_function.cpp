#include "stdafx.h"
#include "hkkim_function.h"

bool readProjectionMatrix(string openCalibFile, cv::Mat& projectionMatrix)
{
	cv::Mat cameraMatrix(3,4,CV_32F,cv::Scalar::all(0));
	cv::Mat R0_rect(4,4,CV_32F,cv::Scalar::all(0));
	R0_rect.at<float>(3,3) = 1.0;
	cv::Mat velo2camera(4,4,CV_32F,cv::Scalar::all(0));
	velo2camera.at<float>(3,3) = 1.0;

	std::ifstream inputFile;
	inputFile.open(openCalibFile.c_str(), std::ios_base::in);
	if (!inputFile.is_open())
	{
		fprintf(stderr, "cannot open the calibration file: %s!\n", openCalibFile.c_str());
		return false;
	}

	while (!inputFile.eof())
	{
		std::string line_tmp;
		std::getline(inputFile, line_tmp);
		std::istringstream inputString(line_tmp);
		std::string tag;
		inputString >> tag;
		if (tag == "P2:") // Projection matrix for left color camera in rectified coordinates(3x4)
		{
			inputString >> cameraMatrix.at<float>(0, 0) >> cameraMatrix.at<float>(0, 1) >> cameraMatrix.at<float>(0, 2) >> cameraMatrix.at<float>(0, 3)
				>> cameraMatrix.at<float>(1, 0) >> cameraMatrix.at<float>(1, 1) >> cameraMatrix.at<float>(1, 2) >> cameraMatrix.at<float>(1, 3)
				>> cameraMatrix.at<float>(2, 0) >> cameraMatrix.at<float>(2, 1) >> cameraMatrix.at<float>(2, 2) >> cameraMatrix.at<float>(2, 3);
		}

		if (tag == "R0_rect:") // Rotation from non-rectified to rectified camera coordinate system (3x3)
		{
			inputString >> R0_rect.at<float>(0, 0) >> R0_rect.at<float>(0, 1) >> R0_rect.at<float>(0, 2)
				>> R0_rect.at<float>(1, 0) >> R0_rect.at<float>(1, 1) >> R0_rect.at<float>(1, 2)
				>> R0_rect.at<float>(2, 0) >> R0_rect.at<float>(2, 1) >> R0_rect.at<float>(2, 2);
		}

		if (tag == "Tr_velo_to_cam:") // Rigid transformation from Velodyne to (non-rectified) camera coordinates (3x4)
		{
			inputString >> velo2camera.at<float>(0, 0) >> velo2camera.at<float>(0, 1) >> velo2camera.at<float>(0, 2) >> velo2camera.at<float>(0, 3)
				>> velo2camera.at<float>(1, 0) >> velo2camera.at<float>(1, 1) >> velo2camera.at<float>(1, 2) >> velo2camera.at<float>(1, 3)
				>> velo2camera.at<float>(2, 0) >> velo2camera.at<float>(2, 1) >> velo2camera.at<float>(2, 2) >> velo2camera.at<float>(2, 3);
		}
	}
	inputFile.close();

	projectionMatrix = cameraMatrix*R0_rect*velo2camera;

	//std::cout << "in the reading, the projectionMatrix = \n " << projectionMatrix << std::endl;
}

Mat KITTI_ReadColorImage(string openImagePath)
{
	Mat LeftOrgColorImage = imread(openImagePath, ImreadModes::IMREAD_ANYCOLOR || ImreadModes::IMREAD_ANYDEPTH);
	return LeftOrgColorImage;
}


void KITTI_ReadVelodyne(string openLidarPath, Size imageSize, Mat projectionMatrix, RangeImage outputDepth)
{
	// allocate 4MB buffer
	int32_t num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	FILE *stream;
	stream = fopen(openLidarPath.c_str(), "rb");
	//stream = fopen(LIDAR_PATH, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	//printf("%d\n", num);

	vector<PointCloud> point_cloud;

	for (int32_t i = 0; i < num; i++) {
		point_cloud.push_back(PointCloud(*px, *py, *pz, *pr));
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(stream);

	double max_distance = 0;
	int el_num = 0;
	for (auto it = point_cloud.begin(); it != point_cloud.end(); it++)
	{
		if (it->px < 0)
			continue;

		Mat cur(4, 1, CV_32FC1, Scalar(0));
		cur.at<float>(0) = it->px;
		cur.at<float>(1) = it->py;
		cur.at<float>(2) = it->pz;
		cur.at<float>(3) = 1.000000e+00;
		float fReflcVal = it->pr * 255;

		Mat Image_coord = projectionMatrix * cur;
		float Image_coord0 = Image_coord.at<float>(0);
		float Image_coord1 = Image_coord.at<float>(1);
		float Image_coord2 = Image_coord.at<float>(2);

		int pos_col = round(Image_coord.at<float>(0) / Image_coord.at<float>(2));
		int pos_row = round(Image_coord.at<float>(1) / Image_coord.at<float>(2));

		if (pos_col >= imageSize.width || pos_row >= imageSize.height || pos_col < 0 || pos_row < 0)
			continue;

		float fDepth = sqrt(it->px * it->px + it->py * it->py + it->pz * it->pz) / 80 * 255;
		outputDepth.floatRangeImage.at<float>(pos_row, pos_col) = fDepth;

		// This function adds RGB color to points in the point cloud based on each point's refelctivity.
		// Blue: Low reflectivity, Yellow/Green: Medium reflectivity, Red: High reflectivity
		float curr_intensity = it->pr * 255;

		float intensity_range = 255; //any intensity value above this value will be red
		float wavelength;
		float min_wavelength = 470; // used to discard overtly blue and purple points that are invisible due to the black background

		if (curr_intensity <= intensity_range)
			wavelength = curr_intensity / intensity_range * (780 - min_wavelength) + min_wavelength;
		else
			wavelength = 780;

		if ((wavelength >= 380) && (wavelength < 440)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = (-(wavelength - 440) / (440 - 380)) * 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 255;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = (-(wavelength - 440) / (440 - 380)) * 255;
		}
		else if ((wavelength >= 440) && (wavelength<490)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = ((wavelength - 440) / (490 - 440)) * 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 255;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = ((wavelength - 440) / (490 - 440)) * 255;
		}
		else if ((wavelength >= 490) && (wavelength<510)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = (-(wavelength - 510) / (510 - 490)) * 255;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = (-(wavelength - 510) / (510 - 490)) * 255;
		}
		else if ((wavelength >= 510) && (wavelength<580)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = ((wavelength - 510) / (580 - 510)) * 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 0;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = ((wavelength - 510) / (580 - 510)) * 255;

		}
		else if ((wavelength >= 580) && (wavelength<645)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = (-(wavelength - 645) / (645 - 580)) * 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 0;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = (-(wavelength - 645) / (645 - 580)) * 255;
		}
		else if ((wavelength >= 645) && (wavelength < 781)) {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = 255;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 0;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = 255;
		}
		else {
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[2] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[1] = 0;
			outputDepth.ColorReflectionImage.at<Vec3b>(pos_row, pos_col)[0] = 0;
			outputDepth.GrayReflectionImage.at<uchar>(pos_row, pos_col) = 0;
		}
	}

	free(data);
	point_cloud.clear();
}


void floatDepthImage2GrayImage(cv::Mat floatDepth, cv::Mat GrayImage)
{
	int imw = floatDepth.size().width;
	int imh = floatDepth.size().height;

	printf("%d %d\n", imw, imh);

	float max_distance = 0;
	float min_distance = 1000;

	for (int h = 0; h < imh; h++)
	{
		for (int w = 0; w < imw; w++)
		{
			float tmp = floatDepth.at<float>(h, w);
			printf("%f\n", tmp);
			if (max_distance < tmp)
			{
				max_distance = tmp;
			}

			if (min_distance > tmp && tmp>0)
			{
				min_distance = tmp;
			}
		}
	}
	printf("max = %f, min = %f\n", max_distance, min_distance);

	for (int h = 0; h < imh; h++)
	{
		for (int w = 0; w < imw; w++)
		{
			float depth = 1000 - floatDepth.at<float>(h, w);

			float temp1 = depth / 1000 * 255;
			int temp2 = (int)(temp1 + 1);
			//printf("%d\n", temp2);

			GrayImage.at<uchar>(h, w) = (unsigned char)temp2;
		}
	}
}
