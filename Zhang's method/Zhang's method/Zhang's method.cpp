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
	vector<vector<Point3f>> objectPoints;
	vector<vector<Point3f>> imagePoints;
	Size size = Size(9, 6), imageSize;//图像尺寸
	Mat cameraMatrix = Mat(3, 3, CV_32FC1);
	Mat distCoeffs = Mat(1, 5, CV_32FC1);
	vector<Mat> rvecs, tvecs;//旋转向量，平移向量
	int flags = 0;
	vector<Point2f> corners;//内角点图像坐标位置
	string s;

	ifstream path("image_index.txt");
	vector<vector<Mat>> Mat1;
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
		vector<Point3f>image_point;
		vector<Mat> Matt;
		Mat m = Mat(3, 1, CV_32FC1);
		for (int i = 0; i < 54; i++)//角点齐次化并将其矩阵化
		{
			Point3f point;
			point.x = corners[i].x;
			point.y = corners[i].y;
			point.z = 1;

			m.at<float>(0, 0) = point.x;
			m.at<float>(1, 0) = point.y;
			m.at<float>(2, 0) = point.z;

			Matt.push_back(m);
			image_point.push_back(point);
		}
		Mat1.push_back(Matt);
		imagePoints.push_back(image_point);
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
				point1.z = 1;//三维点齐次化
				point.push_back(point1);
			}
		}
		objectPoints.push_back(point);
	}
	
	vector<vector<Mat>> Mat2;
	for (int i = 0; i < count; i++)//将三维点化为矩阵形式
	{
		vector<Mat> Matt;
		for (int j = 0; j < 54; j++)
		{
			Mat m = Mat(3, 1, CV_32FC1);
			m.at<float>(0, 0) = objectPoints[i][j].x;
			m.at<float>(1, 0) = objectPoints[i][j].y;
			m.at<float>(2, 0) = objectPoints[i][j].z;

			Matt.push_back(m);
		}
		Mat2.push_back(Matt);
	}
	

	//求解矩阵H
	vector<Mat> H;
	for (int i = 0; i < count; i++)
	{
		Mat H1(108, 9, CV_32FC1);
		for (int j = 0; j < 54; j++)
		{
			Mat H2(1, 9, CV_32FC1);
			Mat H3(1, 9, CV_32FC1);
		
			H2.at<float>(0, 0) = -objectPoints[i][j].x;
			H2.at<float>(0, 1) = -objectPoints[i][j].y;
			H2.at<float>(0, 2) = -1;
			H2.at<float>(0, 3) = 0;
			H2.at<float>(0, 4) = 0;
			H2.at<float>(0, 5) = 0;
			H2.at<float>(0, 6) = objectPoints[i][j].x*imagePoints[i][j].x;
			H2.at<float>(0, 7) = objectPoints[i][j].y*imagePoints[i][j].x;
			H2.at<float>(0, 8) = imagePoints[i][j].x;

			H3.at<float>(0, 0) = 0;
			H3.at<float>(0, 1) = 0;
			H3.at<float>(0, 2) = 0;
			H3.at<float>(0, 3) = -objectPoints[i][j].x;
			H3.at<float>(0, 4) = -objectPoints[i][j].y;
			H3.at<float>(0, 5) = -1;
			H3.at<float>(0, 6) = objectPoints[i][j].x*imagePoints[i][j].y;
			H3.at<float>(0, 7) = objectPoints[i][j].y*imagePoints[i][j].y;
			H3.at<float>(0, 8) = imagePoints[i][j].y;
			
			H2.copyTo(H1.row(j));
			H3.copyTo(H1.row(108 - j - 1));
		}
		
		Mat M;
		Mat Z = Mat::zeros(108, 1, CV_32FC1);
		SVD::solveZ(H1, M);
		
		Mat h(3, 3, CV_32FC1);
		h.at<float>(0, 0) = M.at<float>(0, 0);
		h.at<float>(0, 1) = M.at<float>(1, 0);
		h.at<float>(0, 2) = M.at<float>(2, 0);
		h.at<float>(1, 0) = M.at<float>(3, 0);
		h.at<float>(1, 1) = M.at<float>(4, 0);
		h.at<float>(1, 2) = M.at<float>(5, 0);
		h.at<float>(2, 0) = M.at<float>(6, 0);
		h.at<float>(2, 1) = M.at<float>(7, 0);
		h.at<float>(2, 2) = M.at<float>(8, 0);
		

		H.push_back(h);

	}
	
	//构造V矩阵,用来解b,求出B中未知量
	Mat B(6, 1, CV_32FC1);

	Mat V11 = Mat(1, 6, CV_32FC1);
	Mat V12 = Mat(1, 6, CV_32FC1);
	Mat V22 = Mat(1, 6, CV_32FC1);
	Mat V0 = Mat(1, 6, CV_32FC1);
	Mat V = Mat(108, 6, CV_32FC1);
	for (int i = 0; i < count; i++)
	{
		V12.at<float>(0, 0) = H[i].at<float>(0, 0)*H[i].at<float>(0, 1);
		V12.at<float>(0, 1) = H[i].at<float>(0, 0)*H[i].at<float>(1, 1) + H[i].at<float>(1, 0)*H[i].at<float>(0, 1);
		V12.at<float>(0, 2) = H[i].at<float>(1, 0)*H[i].at<float>(1, 1);
		V12.at<float>(0, 3) = H[i].at<float>(2, 0)*H[i].at<float>(0, 1) + H[i].at<float>(0, 0)*H[i].at<float>(2, 1);
		V12.at<float>(0, 4) = H[i].at<float>(2, 0)*H[i].at<float>(1, 1) + H[i].at<float>(1, 0)*H[i].at<float>(2, 1);
		V12.at<float>(0, 5) = H[i].at<float>(2, 0)*H[i].at<float>(2, 1);

		V11.at<float>(0, 0) = H[i].at<float>(0, 0)*H[i].at<float>(0, 0);
		V11.at<float>(0, 1) = H[i].at<float>(0, 0)*H[i].at<float>(1, 0) + H[i].at<float>(1, 0)*H[i].at<float>(0, 0);
		V11.at<float>(0, 2) = H[i].at<float>(1, 0)*H[i].at<float>(1, 0);
		V11.at<float>(0, 3) = H[i].at<float>(2, 0)*H[i].at<float>(0, 0) + H[i].at<float>(0, 0)*H[i].at<float>(2, 0);
		V11.at<float>(0, 4) = H[i].at<float>(2, 0)*H[i].at<float>(1, 0) + H[i].at<float>(1, 0)*H[i].at<float>(2, 0);
		V11.at<float>(0, 5) = H[i].at<float>(2, 0)*H[i].at<float>(2, 0);

		V22.at<float>(0, 0) = H[i].at<float>(0, 1)*H[i].at<float>(0, 1);
		V22.at<float>(0, 1) = H[i].at<float>(0, 1)*H[i].at<float>(1, 1) + H[i].at<float>(1, 1)*H[i].at<float>(0, 1);
		V22.at<float>(0, 2) = H[i].at<float>(1, 1)*H[i].at<float>(1, 1);
		V22.at<float>(0, 3) = H[i].at<float>(2, 1)*H[i].at<float>(0, 1) + H[i].at<float>(0, 1)*H[i].at<float>(2, 1);
		V22.at<float>(0, 4) = H[i].at<float>(2, 1)*H[i].at<float>(1, 1) + H[i].at<float>(1, 1)*H[i].at<float>(2, 1);
		V22.at<float>(0, 5) = H[i].at<float>(2, 1)*H[i].at<float>(2, 1);


		V0 = V11 - V22;
		V12.copyTo(V.row(i));
		V0.copyTo(V.row(2 * count - i - 1));
	}
	SVD::solveZ(V, B);
	cout << "内参："<<B.at<float>(0, 0) << " " << B.at<float>(1, 0) << " " << B.at<float>(2, 0) << " " << B.at<float>(3, 0) << " " << B.at<float>(4, 0) << " " << B.at<float>(5, 0)<<endl;
	
	//求解内参

	float v0, aa, bb, cc, u0;

	v0 = (B.at<float>(1, 0)*B.at<float>(3, 0) - B.at<float>(0, 0)*B.at<float>(4, 0)) / (B.at<float>(0, 0)*B.at<float>(2, 0) - B.at<float>(1, 0)*B.at<float>(1, 0));
	float scale = B.at<float>(5, 0) - (B.at<float>(3, 0)*B.at<float>(3, 0) + v0*(B.at<float>(1, 0)*B.at<float>(3, 0) - B.at<float>(0, 0)*B.at<float>(4, 0))) / B.at<float>(0, 0);
	aa = sqrt(scale / B.at<float>(0, 0));
	bb = sqrt(scale*B.at<float>(0, 0) / (B.at<float>(0, 0)*B.at<float>(2, 0) - B.at<float>(1, 0)*B.at<float>(1, 0)));
	cc = -(B.at<float>(1, 0)*aa*aa*bb / scale);
	u0 = cc*v0 / bb - B.at<float>(3, 0)*aa*aa / scale;

	cameraMatrix.at<float>(0, 0) = aa;
	cameraMatrix.at<float>(0, 1) = cc;
	cameraMatrix.at<float>(0, 2) = u0;
	cameraMatrix.at<float>(1, 0) = 0;
	cameraMatrix.at<float>(1, 1) = bb;
	cameraMatrix.at<float>(1, 2) = v0;
	cameraMatrix.at<float>(2, 0) = 0;
	cameraMatrix.at<float>(2, 1) = 0;
	cameraMatrix.at<float>(2, 2) = 1;

	cout << "外参："<<aa << " " << cc << " " << u0 << " " << v0 << " " << bb<<endl;
	//求解外参
	system("Pause");



}