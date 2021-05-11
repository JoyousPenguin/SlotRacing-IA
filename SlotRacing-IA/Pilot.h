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

	std::vector<cv::Rect2d> StraightRect;
	std::vector<int> StraightVecX;
	std::vector<int> StraightVecY;

	std::vector<cv::Rect2d> TurnRect;
	std::vector<int> TurnVecX;
	std::vector<int> TurnVecY;

	std::vector<cv::Rect2d> TightTurnRect;
	std::vector<int> TightTurnVecX;
	std::vector<int> TightTurnVecY;
	

	SerialPort* bridge;

	void CompleteVect(cv::Mat& SectionsImg, std::vector<cv::Rect2d>& SectionsRect, std::vector<int>& SectionsVecX, std::vector<int>& SectionsVecY);


	bool CheckPath(cv::Point p, std::vector<int>SectionVecX, std::vector<int>SectionVecY);


public:
	Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask);

	~Pilot();

	void train(cv::Mat& StraightSections, cv::Mat& TurnSections, cv::Mat& TightTurnSections);

	void drive(cv::Mat carPath);
};

