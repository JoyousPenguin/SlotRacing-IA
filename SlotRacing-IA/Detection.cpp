#include "Detection.h"


Detection::Detection()
{
    initBackSub();
};


Detection::~Detection()
{};


void Detection::initBackSub()
{
    int history = 500;      //how many last frames affect the background model.
    double varTreshold = 400;
    bool detectShadow = false;

    d_BackSub = cv::createBackgroundSubtractorMOG2(history, varTreshold, detectShadow);
}


std::vector<std::vector<cv::Point>> Detection::BackgroundSubstraction(cv::Mat& stream, double learningRate)
{
    //double learningRate = 0.01;


    cv::Mat fgmask;
    //fgmask.size() = image.size();
    d_BackSub->apply(stream, fgmask, learningRate);

    cv::Mat fgmask_tresh;
    //fgmask_tresh.size() = image.size();
    threshold(fgmask, fgmask_tresh, 250, 255, cv::THRESH_BINARY);

    cv::Mat dilated;
    //dilated.size() = image.size();
    dilate(fgmask_tresh, dilated, cv::Mat(), cv::Point(-1, -1), 5, cv::BORDER_CONSTANT);

    cv::Mat eroded;
    //eroded.size() = image.size();
    erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1), 6, cv::BORDER_CONSTANT);

    std::vector<std::vector<cv::Point>> Edges;
    cv::findContours(eroded, Edges, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


    int biggestArea = 0;
    int biggestAreaIdx = -1;

    for (int i = 0; i < Edges.size(); i++)
    {
        if (cv::contourArea(Edges[i]) > biggestArea)
        {
            biggestAreaIdx = i;
            biggestArea = cv::contourArea(Edges[i]);
        }
    }

    std::vector<std::vector<cv::Point>> FinalEdges;

    if(biggestAreaIdx != -1)
        FinalEdges.push_back(Edges[biggestAreaIdx]);



    fgmask.release();
    fgmask_tresh.release();
    dilated.release();
    eroded.release();


    return FinalEdges;
};


void Detection::GetView(cv::Mat& input, cv::Mat& output, cv::Mat& M, cv::Rect2d& r, cv::Mat& mask)
{
    cv::Mat flat, bitAndImg;


    cv::warpPerspective(input, flat, M, input.size());

    cv::Mat im(flat);
    cv::Mat resized = im(r);

    bitwise_and(resized, resized, bitAndImg, mask);

    cv::resize(bitAndImg, output, cv::Size(), 2, 2, cv::INTER_LINEAR);

    resized.release();


    flat.release();
    bitAndImg.release();

};