// MOD.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "hkkim_function.h"

int main()
{
	Mat projectionMatrix(3, 4, CV_32F, cv::Scalar::all(0));
	readProjectionMatrix(KITTI_Calib_PATH, projectionMatrix);

	char LIDAR_PATH[255], IMAGE_PATH[255];// , LIDAR_DATA[255];
	int frame_num = FILE_START -1;
	do
	{
		frame_num++;
		if (frame_num == FILE_END)
		{
			cout << "==== END OF FILE ====\n" << endl;
			break;
		}
		sprintf(IMAGE_PATH, KITTI_I_PATH "/" FILE_NAME "_%06d." IMAGE_FILE_FORMAT, frame_num);
		sprintf(LIDAR_PATH, KITTI_L_PATH "/" FILE_NAME "_%06d." LIDAR_FILE_FORMAT, frame_num);
		
		Mat LeftOrgColorImage = KITTI_ReadColorImage(IMAGE_PATH);
		Size IMAGE_SIZE(LeftOrgColorImage.size().width, LeftOrgColorImage.size().height);
		RangeImage Depth(IMAGE_SIZE);

		
		
		//KITTI_ReadVelodyne(LIDAR_PATH, IMAGE_SIZE, Project_Rect, Rotation_Rect, Calib_Velo_to_Cam, Depth);
		KITTI_ReadVelodyne(LIDAR_PATH, IMAGE_SIZE, projectionMatrix, Depth);
		Mat DepthMapGray(IMAGE_SIZE, CV_8UC1, Scalar(0));

		Depth.floatRangeImage.convertTo(DepthMapGray, CV_8UC1);
		
		imshow("LeftOrgColorImage", LeftOrgColorImage);
		imshow("DepthMapGray", DepthMapGray);
		imshow("ReflectivityColor", Depth.ColorReflectionImage);
		//imshow("GrayReflectionImage", Depth.GrayReflectionImage);


	} while (waitKey(1) != 27);

	return 0;
}
