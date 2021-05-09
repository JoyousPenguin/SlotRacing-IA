#pragma once

#include "Detection.h"
#include "Transformation.h"
#include "SerialPort.h"


class Pilot:
	Detection, Transformation
{
private:
	cv::Mat p_M;
	cv::Rect2d p_cadre;
	cv::Mat p_mask;

	cv::VideoCapture p_cap;
	

	SerialPort* bridge;

public:
	Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask);

	~Pilot();

	void drive(cv::Mat carPath);
};

