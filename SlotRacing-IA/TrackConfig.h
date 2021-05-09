#pragma once
#include "Transformation.h"
#include "Detection.h"





class TrackConfig :
    Detection, Transformation
{
private:
    cv::Mat tc_M;
    cv::Rect2d tc_cadre;
    cv::Mat tc_mask;

    cv::VideoCapture tc_cap;

    std::vector<cv::Point2f> filtered_ordered_point_path;

    cv::Mat tc_StraightSections, tc_TurnSections, tc_TightTurnSections;

    void selection(cv::Mat& image, cv::Mat& output, int nbre);

    void getPath(std::vector<cv::Point> points, cv::Mat& drawing_path);

public:
    TrackConfig(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask);

    ~TrackConfig();

    void SectionsSelecter(cv::Mat& image, cv::Mat& StraightSections, cv::Mat& TurnSections, cv::Mat& TightTurnSections);

    void SavePath(cv::Mat& CarPath);

    void DecomposePath(cv::Size p, std::vector<std::pair<int, int>> Track);
};

