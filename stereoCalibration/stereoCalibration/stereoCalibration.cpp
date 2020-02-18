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

	int count = 0;
	vector<vector<Point3f>> objectPoints1;
	vector<vector<Point2f>> imagePoints1;
	Size size = Size(9, 6), imageSize;
	Mat cameraMatrix1 = Mat(3, 3, CV_32FC1);
	Mat distCoeffs1 = Mat(1, 5, CV_32FC1);
	vector<Mat> rvecs1, tvecs1;
	int flags = 0;
	vector<Point2f> corners;//内角点图像坐标位置
	string s;

	ifstream path("calibration1.txt");
	while (getline(path, s))
	{
		count++;
		Mat imageInput = imread(s);
		if (count == 1)
		{
			imageSize.width = imageInput.cols;
			imageSize.height = imageInput.rows;
		}
		findChessboardCorners(imageInput, size, corners);
		imagePoints1.push_back(corners);
		drawChessboardCorners(imageInput, size, corners, true);
		
		

	}



	Size square_size = Size(10, 10);
	for (int i = 0; i < count; i++)
	{
		vector<Point3f>point;
		for (int j = 0; j < size.height; j++)
		{
			for (int k = 0; k < size.width; k++)
			{
				Point3f point1;
				point1.x = j *square_size.width;
				point1.y = k * square_size.height;
				point1.z = 0;
				point.push_back(point1);
			}
		}
		objectPoints1.push_back(point);
	}

	calibrateCamera(objectPoints1, imagePoints1, imageSize, cameraMatrix1, distCoeffs1, rvecs1, tvecs1, 0);


	
	count = 0;
	vector<vector<Point3f>> objectPoints2;
	vector<vector<Point2f>> imagePoints2;
	Mat cameraMatrix2 = Mat(3, 3, CV_32FC1);
	Mat distCoeffs2 = Mat(1, 5, CV_32FC1);
	vector<Mat> rvecs2, tvecs2;
    ifstream path2("calibration2.txt");
	while (getline(path2, s))
	{
		count++;
		Mat imageInput = imread(s);
		if (count == 1)
		{
			imageSize.width = imageInput.cols;
			imageSize.height = imageInput.rows;
		}
		findChessboardCorners(imageInput, size, corners);
		imagePoints2.push_back(corners);
		drawChessboardCorners(imageInput, size, corners, true);
		
		

	}
	for (int i = 0; i < count; i++)
	{
		vector<Point3f>point;
		for (int j = 0; j < size.height; j++)
		{
			for (int k = 0; k < size.width; k++)
			{
				Point3f point1;
				point1.x = j *square_size.width;
				point1.y = k * square_size.height;
				point1.z = 0;
				point.push_back(point1);
			}
		}
		objectPoints2.push_back(point);
	}

	calibrateCamera(objectPoints2, imagePoints2, imageSize, cameraMatrix2, distCoeffs2, rvecs2, tvecs2, 0);


	
	
	//立体标定
	
	Mat R = Mat(3, 3, CV_32FC1);
	Mat T = Mat(3, 1, CV_32FC1);
	Mat E = Mat(3, 3, CV_32FC1);
	Mat F = Mat(3, 3, CV_32FC1);
	stereoCalibrate(objectPoints1,imagePoints1,imagePoints2,cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2,imageSize,R,T,E,F);//求出R、T、E、F
	ofstream out("result.txt");
	out <<"R:"<< R << endl;
	out << "T:"<<T << endl <<"E:"<< E << endl <<"F:"<< F << endl;
	
	
}
