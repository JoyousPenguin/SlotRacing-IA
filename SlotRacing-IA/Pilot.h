#pragma once

#include "Detection.h"
#include "Transformation.h"
#include "SerialPort.h"


class Pilot:
	Detection
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

	cv::Mat StraightSections, TurnSections, TightTurnSections;
	
	std::vector<cv::Point2f> filtered_ordered_point_path;

	//contains a mask of the path of the track
	cv::Mat CarPath;

	//list of section with size
	std::vector<std::pair<int, int>> Track;

	SerialPort* bridge;

	void CompleteVect(cv::Mat& SectionsImg, std::vector<cv::Rect2d>& SectionsRect, std::vector<int>& SectionsVecX, std::vector<int>& SectionsVecY);


	bool CheckPath(cv::Point p, std::vector<int>SectionVecX, std::vector<int>SectionVecY);


	void selection(cv::Mat& image, cv::Mat& output, int nbre);


	void getPath(std::vector<cv::Point> points, cv::Mat& drawing_path);

public:
	Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask);


	~Pilot();


	void SectionsSelecter(cv::Mat& image);


	void SavePath();


	void DecomposePath(cv::Size p);


	void train();


	void drive();
};

