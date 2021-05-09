#include "Pilot.h"

Pilot::Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask)
{
    this->p_M = M;
    this->p_cadre = r;
    this->p_mask = mask;

    this->p_cap = cap;

    const char* portName = "\\\\.\\COM9";
    bridge = new SerialPort(portName);

    if (bridge->isConnected())
        std::cout << "connected to Arduino" << std::endl;
    else
        std::cout << "ERROR: not connected to Arduino" << std::endl;


};


Pilot::~Pilot()
{
    p_M.release();
    p_mask.release();

    p_cap.release();
};


void Pilot::drive(cv::Mat carPath)
{
    cv::Mat image, stream;

    bool state = p_cap.read(image);
    Transformation::GetView(image, stream, p_M, p_cadre, p_mask);
    image.release();

    cv::Mat trackMask;
    cv::resize(p_mask, trackMask, cv::Size(), 2.0, 2.0);


    //car 1 -- IA
    cv::Mat trackCar1;
    cv::bitwise_and(stream, stream, trackCar1, carPath);

    double LearningRate = 0.01;
    std::vector<std::vector<cv::Point>> car1;
    car1 = Detection::BackgroundSubstraction(trackCar1 ,LearningRate);


    std::string wind_Vid = "Stream";
    cv::namedWindow(wind_Vid, cv::WINDOW_AUTOSIZE);

    std::cout << "==== START ====" << std::endl;

    while (state)
    {
        state = p_cap.read(image);
        Transformation::GetView(image, stream, p_M, p_cadre, p_mask);
        image.release();

        cv::bitwise_and(stream, stream, trackCar1, carPath);
        car1 = Detection::BackgroundSubstraction(trackCar1, LearningRate);

        //processing car1 to detect position of car



        cv::imshow(wind_Vid, stream);
        cv::imshow("car1 IA", trackCar1);
        stream.release();
        trackCar1.release();


        if (cv::waitKey(10) == 27)
            state = false;
    }
    
    trackMask.release();
    
};