#include "stdafx.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>  
#include <fstream>  

using namespace cv;
using namespace std;

void main()
{
	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>>imagePoints;
	Size imageSize, size1(10, 10);
	Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;

	ifstream path("image_index.txt");
	string s;
	int count = 0;
	Size size(9, 6);
	vector<Point2f>corners;
	while (getline(path, s))
	{
		Mat image = imread(s);
		imageSize.width = image.cols;
		imageSize.height = image.rows;
		findChessboardCorners(image, size, corners);
		drawChessboardCorners(image, size, corners, true);
		imagePoints.push_back(corners);
		//imshow("", image);
		count++;
		//waitKey(700);
	}

	for (int i = 0; i < count; i++)
	{
		vector<Point3f>point;
		for (int j = 0; j < size.height; j++)
		{
			for (int k = 0; k < size.width; k++)
			{
				Point3f point1;
				point1.x = k*size1.width;
				point1.y = j*size1.height;
				point1.z = 0;
				point.push_back(point1);
			}
		}
		objectPoints.push_back(point);

	}
	calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);

	ifstream path1("image_index.txt");
	while (getline(path1, s))
	{
		Mat img = imread(s);
		Mat img1;
		undistort(img, img1, cameraMatrix, distCoeffs);
		imshow("矫正前", img);
		waitKey(1000);
		imshow("矫正后", img1);
		waitKey(1000);
	}

	return;
}
