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


void Pilot::CompleteVect(cv::Mat& SectionsImg, std::vector<cv::Rect2d>& SectionsRect, std::vector<int>& SectionsVecX, std::vector<int>& SectionsVecY)
{
    std::vector<std::vector<cv::Point>> Contours;
    cv::findContours(SectionsImg, Contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    for (int i = 0; i < Contours.size(); i++)
    {
        cv::Rect2d r = cv::boundingRect(Contours[i]);
        SectionsRect.push_back(r);

        SectionsVecX.push_back(r.x);
        SectionsVecX.push_back(r.x + r.width);

        SectionsVecY.push_back(r.y);
        SectionsVecY.push_back(r.y + r.height);
    }
}


void Pilot::train(cv::Mat& StraightSections, cv::Mat& TurnSections, cv::Mat& TightTurnSections)
{
    /*
    * à partir des masques on choppe les boundingRect
    * 
    * dans vecteur RectstraightSectionX on stock
    *   minVAL1 maxVAL1 minVAL2 maxVAL2 ...
    * 
    */

    CompleteVect(StraightSections, StraightRect, StraightVecX, StraightVecY);

    CompleteVect(TurnSections, TurnRect, TurnVecX, TurnVecY);

    CompleteVect(TightTurnSections, TightTurnRect, TightTurnVecX, TightTurnVecY);


    /*for (int i = 0; i < StraightVecX.size(); i++)
    {

    }*/

    cv::waitKey(0);
};


bool Pilot::CheckPath(cv::Point p, std::vector<int>SectionVecX, std::vector<int>SectionVecY)
{
    bool section = false;
    std::vector<int> SectionVecI;

    for (int i = 0; i < SectionVecX.size() - 1; i += 2)
    {
        if (SectionVecX[i] < p.x && p.x < SectionVecX[i + 1])
        {
            //std::cout <<"Stat 1 "<< SectionVecX[i]<<" < "<<p.x<<" < "<< SectionVecX[i + 1] << std::endl;
            section = true;
            SectionVecI.push_back(i);
        }
    }

    if (section)
    {
        section = false;
        for (int j = 0; j < SectionVecI.size(); j++)
        {
            if (SectionVecY[SectionVecI[j]] < p.y && p.x < SectionVecY[SectionVecI[j] + 1])
            {
                //std::cout <<"Stat 2 "<< SectionVecY[SectionVecI[j]] << " < " << p.y << " < " << SectionVecY[SectionVecI[j] + 1] << std::endl;
                section = true;
            }
        }
    }

    SectionVecI.clear();
    return section;
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

    char data = 0x32;
    char prvdata = 0x00;

    while (state)
    {
        state = p_cap.read(image);
        Transformation::GetView(image, stream, p_M, p_cadre, p_mask);
        image.release();

        cv::bitwise_and(stream, stream, trackCar1, carPath);
        car1 = Detection::BackgroundSubstraction(trackCar1, LearningRate);
        cv::Point p = cv::Point(-1,-1);
        for (int i = 0; i < car1.size(); i++)
        {
            cv::Rect2d r = boundingRect(car1[i]);
            rectangle(stream, r, cv::Scalar(0, 255, 0), 2);
            p = cv::Point(r.x + r.width / 2, r.y + r.height / 2);
            cv::circle(stream, p, 2, cv::Scalar(0, 0, 255));
        }


        //processing car1 to detect position of car
        bool straight = false;

        bool turn = false;

        bool tightturn = false;

        for (int i = 0; i < StraightRect.size(); i++)
        {
            cv::rectangle(stream, StraightRect[i], cv::Scalar(255, 0, 0), 2);
        }
        for (int i = 0; i < TurnRect.size(); i++)
        {
            cv::rectangle(stream, TurnRect[i], cv::Scalar(0, 255, 0), 2);
        }
        for (int i = 0; i < TightTurnRect.size(); i++)
        {
            cv::rectangle(stream, TightTurnRect[i], cv::Scalar(0, 0, 255), 2);
        }


        if (p != cv::Point(-1, -1))
        {
            std::cout << "New Frame" << std::endl;
            straight = CheckPath(p, StraightVecX, StraightVecY);
           // cv::waitKey(0);
            turn = CheckPath(p, TurnVecX, TurnVecY);
            //cv::waitKey(0);
            tightturn = CheckPath(p, TightTurnVecX, TightTurnVecY);
            //cv::waitKey(0);

            if (straight)
            {
                circle(stream, cv::Point(10, 10), 5, cv::Scalar(255, 0, 0), -1);
                data = 0x64;
            }

            if (turn)
            {
                circle(stream, cv::Point(10, 20), 5, cv::Scalar(0, 255, 0), -1);
                data = 0x50;
            }     

            if (tightturn)
            {
                circle(stream, cv::Point(10, 30), 5, cv::Scalar(0, 0, 255), -1);
                data = 0x46;
            }

            if (data != prvdata)
            {
                bridge->writeSerialPort(&data, 1);
                prvdata = data;
            }



        }

        

        
        







        cv::imshow(wind_Vid, stream);
        cv::imshow("car1 IA", trackCar1);
        stream.release();
        trackCar1.release();


        if (cv::waitKey(10) == 27)
            state = false;
    }
    
    trackMask.release();
    
};