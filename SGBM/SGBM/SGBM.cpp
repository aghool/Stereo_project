

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;
int main()
{
	int minDisparity = 0;
	int numDisparities = 64;//16的整数倍
	int SADWindowSize = 11;//大于1的奇数

	int disp12MaxDiff = 1;
	int preFilterCap = 15, uniquenessRatio = 15;//5-15
	int speckleWindowSize = 100, speckleRange = 2;//50-200/ 1或2
	bool fullDP = false;//默认为false

	Mat left = imread("left01.jpg");
	Mat right = imread("right01.jpg");

	cvtColor(left, left, CV_BGR2GRAY);
	cvtColor(right, right, CV_BGR2GRAY);

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparities, SADWindowSize);

	sgbm->setDisp12MaxDiff(disp12MaxDiff);
	sgbm->setP1(8 * left.channels()*SADWindowSize*SADWindowSize);
	sgbm->setP2(32 * right.channels()*SADWindowSize*SADWindowSize);
	sgbm->setPreFilterCap(preFilterCap);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(speckleWindowSize);
	sgbm->setSpeckleRange(speckleRange);


	Mat image;

	sgbm->compute(left, right, image);
	image.convertTo(image, CV_8U, 255 / (numDisparities*16.));
	imshow("disparity", image);
	waitKey();

}
