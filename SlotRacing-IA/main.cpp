#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>

#include <iostream>
#include <iomanip>


#include "Transformation.h"
#include "TrackConfig.h"
#include "Pilot.h"


cv::Mat M;
cv::Rect2d cadre;
cv::Mat mask;




int main()
{
	std::cout << "Program Start" << std::endl;


    //****************************Capturing Stream*************************************
    cv::VideoCapture cap(1);
    bool state;

    if (cap.isOpened() == false)
    {
        std::cout << "Cannot open flux" << std::endl;
        cv::waitKey(0);
        return -1;
    }

    //wait some seconds to load correctly the image
    cv::Mat image;
    for (int i = 0; i < 240; i++)
    {
        state = cap.read(image);
        cv::waitKey(10);

        if (!state)
            break;
    }

    //****************************Showing video stream before start*************************************

    std::string window_start = "Camera";
    cv::namedWindow(window_start, cv::WINDOW_AUTOSIZE);

    std::cout << "Press SPACE to start" << std::endl;

    while (state)
    {
        state=cap.read(image);
        cv::imshow(window_start, image);
        if (cv::waitKey(50) == ' ')
            break;
    }

    image.release();
    cv::destroyWindow(window_start);

    //****************************Starting process to isolate track*************************************


    Transformation adobe(cap);
    adobe.Process();
    adobe.GetTransformParam(M, cadre, mask);

    //****************************Showing Result of transforation*************************************

    std::cout << "Press SPACE to continue" << std::endl;
        
    std::string wind_stream = "Video";
    cv::namedWindow(wind_stream, cv::WINDOW_AUTOSIZE);

    cv::Mat stream;
    while(adobe.GetView(stream))
    {
        cv::imshow(wind_stream, stream);

        if (cv::waitKey(20) == ' ')
            break;
    }
    cv::destroyWindow(wind_stream);
    stream.release();

    //****************************Select ROI for sections*************************************    
    

    TrackConfig PaulRicard (cap, M, cadre, mask);

    cv::Mat StraightSections, TurnSections, TightTurnSections;
    adobe.GetView(stream);

    PaulRicard.SectionsSelecter(stream, StraightSections, TurnSections, TightTurnSections);



    //****************************Detection of the path*************************************

    cv::Mat carPath;
    PaulRicard.SavePath(carPath);


    //****************************From path and Section get order******************************************


    std::vector<std::pair<int, int>> Track;
    PaulRicard.DecomposePath(stream.size(), Track);
    stream.release();

    //****************************Get car separatly******************************************

    Pilot Hamilton(cap, M, cadre, mask);
    Hamilton.train(StraightSections, TurnSections, TightTurnSections);

    Hamilton.drive(carPath);
   

    /*while (state)
    {




        std::cout << "TOUR" << std::endl;
        video_stream.release();
        stream.release();

        state = cap.read(video_stream);
        TransformView(video_stream, stream, M, cadre, mask);

        //car 1 = IA
        cv::bitwise_and(stream, stream, TrackCar1, carPath);
        car1 = BackgroundSubstraction(BackSubTrackCar1, TrackCar1);





        cv::Point p;
        for (int i = 0; i < car1.size(); i++)
        {
            cv::Rect2d r = boundingRect(car1[i]);
            rectangle(stream, r, cv::Scalar(0, 255, 0), 2);
            p = cv::Point(r.x + r.width / 2, r.y + r.height / 2);
            cv::circle(stream, p, 2, cv::Scalar(0, 0, 255));
        }

        
        cv::Mat black = cv::Mat::zeros(stream.size(), CV_8U);
        cv::Mat Result;
        cv::bitwise_and(black, black, Result, StraightSection);
        if (cv::countNonZero(Result) == 0)
        {
            cv::bitwise_and(black, black, Result, TurnSection);
            if (cv::countNonZero(Result) == 0)
            {
                cv::bitwise_and(black, black, Result, TightTurnSection);
                if (cv::countNonZero(Result) == 0)
                {
                    //val = -1;
                    //data = 0x32;//50
                    continue;
                }
                else
                {
                    data = 0x46; //70
                    std::cout << "TOURNE FORT" << std::endl;
                }
            }
            else
            {
                data = 0x50; //80
                std::cout << "TOURNE" << std::endl;
            }

        }
        else
        {
            data = 0x64; //100
            std::cout << "DROIT" << std::endl;
        }

        if (data != prevData || prevData == 'A')
        {
            bridge->writeSerialPort(&data, 1);

            std::cout << "value sended: " << std::hex << data << std::endl;
            prevData = data;
        }










        cv::imshow(wind_Vid, stream);
        cv::imshow("car1 IA", TrackCar1);

        cv::waitKey(1);

        if (cv::waitKey(10) == 27)
            state = false;

    }*/
   
    //****************************Quit*************************************

    std::cout << "Press any key to exit program" << std::endl;
    cv::waitKey(0);


    M.release();
    mask.release();

    cv::destroyAllWindows();

    return 0;
}
