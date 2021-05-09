#pragma once
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include<fstream> //file management

#include <iostream>



class Transformation
{
private:


	//vector for points used to straighten up the image
	std::vector<cv::Point> t_pointVec;


	//void mouseClick(int event, int x, int y, int, void*);

	void GetClickPoint();

	void GetTransformMat(cv::Mat& frame);

	void ComputeTransformViewParam();

	cv::VideoCapture t_cap;

	cv::Mat t_M;
	cv::Rect2d t_cadre;
	cv::Mat t_mask;

protected:

	void GetView(cv::Mat& input, cv::Mat& output, cv::Mat& M, cv::Rect2d& r, cv::Mat& mask);
	

public:

	Transformation();

	Transformation(cv::VideoCapture cap);

	~Transformation();

	void Process();

	bool GetView(cv::Mat& output);

	void GetTransformParam(cv::Mat& M, cv::Rect2d& r, cv::Mat& mask);

	

	
};

