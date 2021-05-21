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

	const unsigned int INVALID = -1;
	const unsigned int STRAIGHT = 1;
	const unsigned int TURN = 2;
	const unsigned int TIGHTURN = 3;



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

	//contains all the point and the section in wich there are
	std::vector<std::pair<cv::Point2f, int>> PointsSection;

	//Rect of the start point
	cv::Rect2d Startgrid;
	


	SerialPort* bridge;

	void CompleteVect(cv::Mat& SectionsImg, std::vector<cv::Rect2d>& SectionsRect, std::vector<int>& SectionsVecX, std::vector<int>& SectionsVecY);


	bool CheckPath(cv::Point p, std::vector<int>SectionVecX, std::vector<int>SectionVecY);


	void selection(cv::Mat& image, cv::Mat& output, int nbre);


	void getPath(std::vector<cv::Point> points, cv::Mat& drawing_path);


	int getSection(cv::Point2f p);


	int binary_search(const std::vector<std::pair<int, double>>& sorted_vec, double key);

public:
	Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask);


	~Pilot();


	void SectionsSelecter(cv::Mat& image);


	void SavePath();


	void train();


	void drive();
};

