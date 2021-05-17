#pragma once
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>



class Detection
{

private:
	cv::Ptr<cv::BackgroundSubtractor> d_BackSub;

protected:
	void initBackSub();

	std::vector<std::vector<cv::Point>> BackgroundSubstraction(cv::Mat& stream, double learningRate);

	void GetView(cv::Mat& input, cv::Mat& output, cv::Mat& M, cv::Rect2d& r, cv::Mat& mask);

public:
	Detection();

	~Detection();
};

