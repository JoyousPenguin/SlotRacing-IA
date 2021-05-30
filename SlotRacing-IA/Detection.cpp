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
    //cv::imshow("0 - Output BackgroundSubstraction", fgmask);

    cv::Mat fgmask_tresh;
    //fgmask_tresh.size() = image.size();
    threshold(fgmask, fgmask_tresh, 250, 255, cv::THRESH_BINARY);
    //cv::imshow("A - Output Treshold", fgmask_tresh);

    cv::Mat dilated;
    //dilated.size() = image.size();
    dilate(fgmask_tresh, dilated, cv::Mat(), cv::Point(-1, -1), 2, cv::BORDER_CONSTANT);
    //cv::imshow("B - Output Dilated", dilated);

    cv::Mat eroded;
    //eroded.size() = image.size();
    erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1), 3, cv::BORDER_CONSTANT);
    //cv::imshow("B - Output Eroded", eroded);


    std::vector<std::vector<cv::Point>> Edges;
    cv::findContours(eroded, Edges, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


    int biggestArea = 0;
    int biggestAreaIdx = -1;

    //cv::Mat contoursMat = cv::Mat::zeros(eroded.size(), CV_8U);
    //cv::Mat biggestcontoursMat = cv::Mat::zeros(eroded.size(), CV_8U);

    for (int i = 0; i < Edges.size(); i++)
    {
        if (cv::contourArea(Edges[i]) > biggestArea)
        {
            biggestAreaIdx = i;
            biggestArea = cv::contourArea(Edges[i]);
            //cv::drawContours(biggestcontoursMat, Edges, i, cv::Scalar(255), 1, cv::LINE_8);
        }
        //cv::drawContours(contoursMat, Edges, i, cv::Scalar(255), 1, cv::LINE_8);
    }

    //cv::imshow("C - Output contour", contoursMat);

    //cv::imshow("D - Output biggest contour", biggestcontoursMat);

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