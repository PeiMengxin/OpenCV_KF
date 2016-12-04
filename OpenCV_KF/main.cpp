#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;


//

int main()
{
	RNG rng(12345);
	
	std::vector<float> Y;
	std::vector<float> Y_rng;
	std::vector<int> X;
	for (size_t i = 0; i < 300; i++)
	{
		X.push_back(i + 100);
		Y.push_back(sin(float(X[i]) / 30) * 100 + 200);
		Y_rng.push_back(/*sin(float(X[i]) / 30) * 100 + */200 + rng.gaussian(10));
	}

	int winHeight = 500;
	int winWidth = 500;

	const int stateNum = 1;                                      //״ֵ̬4��1����(x,y,��x,��y)
	const int measureNum = 1;                                    //����ֵ2��1����(x,y)	
	KalmanFilter KF(stateNum, measureNum, 0);

	KF.transitionMatrix = *(Mat_<float>(1, 1) << 1);  //ת�ƾ���A
	setIdentity(KF.measurementMatrix);                                             //��������H
	setIdentity(KF.processNoiseCov, Scalar::all(0.1));                            //ϵͳ�����������Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(100));                        //���������������R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //����������Э�������P
	//rng.fill(KF.statePost, RNG::UNIFORM, 0,1);   //��ʼ״ֵ̬x(0)
	
	KF.statePost.at<float>(0) = Y_rng[0];
	
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //��ʼ����ֵx'(0)����Ϊ����Ҫ�������ֵ�����Ա����ȶ���

	namedWindow("kalman");

	Mat image(winHeight, winWidth, CV_8UC3, Scalar(0));

	for (size_t i = 1; i < X.size(); i++)
	{
		//circle(image, Point(X[i], Y[i]), 1, CV_RGB(0, 255, 0), -1);
		circle(image, Point(X[i], Y_rng[i]), 1, CV_RGB(0, 0, 255), -1);

		KF.predict();
		circle(image, Point(X[i], KF.statePre.at<float>(0)), 1, CV_RGB(255, 0, 0), -1);
		
		measurement.at<float>(0) = Y_rng[i];

		KF.correct(measurement);
		circle(image, Point(X[i], KF.statePost.at<float>(0)), 1, CV_RGB(255, 255, 0), -1);
		imshow("kalman", image);
		waitKey(5);
	}

	imshow("kalman", image);

	waitKey(0);
}