#pragma once
#include <opencv2/imgproc.hpp>





class Track
{

protected:
	cv::Mat t_M;
	cv::Mat t_mask;
	cv::Rect t_cadre;


};

