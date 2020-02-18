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
	stereoCalibrate(objectPoints1, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F);//求出R、T、E、F
	ofstream out("result.txt");
	out << "R:" << R << endl;
	out << "T:" << T << endl << "E:" << E << endl << "F:" << F << endl;

	Mat R1, R2, P1, P2, Q;
	//Q将图像平面转换到三维坐标
	stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q);//求出R1,R2,P1,P2
	out << "R1:" << R1 << endl << "R2:" << R2 << endl << "Q:" << Q << endl;

	cout << T << endl;

	Mat map1x, map1y, map2x, map2y;
	initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, map1x, map1y);//mapx,mapy为映射表
	initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, map2x, map2y);
	out << "map1x:" << map1x << endl << "map1y:" << map1y << endl << "map2x:" << map2x << endl << "map2y:" << map2y << endl;




	ifstream pathh("calibration1.txt");
	ifstream pathh1("calibration2.txt");
	string s1;
	vector<Mat> image1, image2;
	while (getline(pathh, s) && getline(pathh1, s1))
	{
		Mat img1, img1r, img2, img2r;
		img1r = imread(s);
		img2r = imread(s1);

		remap(img1r, img1, map1x, map1y, INTER_LINEAR);//remap可嵌入映射表
		remap(img2r, img2, map2x, map2y, INTER_LINEAR);

		image1.push_back(img1);
		image2.push_back(img2);
	}
	for (int i = 0; i < count; i++)
	{
		Mat img1 = image1[i];
		Mat img2 = image2[i];

		double sf;
		int w, h;

		w = imageSize.width;
		h = imageSize.height;
		Mat canvas = Mat(h, w * 2, CV_8UC3);

		Mat area = canvas(Rect(0, 0, w, h));

		resize(img1, area, area.size(), 0, 0, INTER_AREA); //改变图像大小，canvasPart为输出图像



		area = canvas(Rect(w, 0, w, h));
		resize(img2, area, area.size(), 0, 0, INTER_LINEAR);


		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

		imshow("stereoCalibration", canvas);
		waitKey(1000);


	}
}
