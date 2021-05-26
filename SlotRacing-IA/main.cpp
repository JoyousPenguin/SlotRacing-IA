#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>

#include <iostream>
#include <iomanip>


#include "Transformation.h"
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

    Transformation Transfo(cap);
    Transfo.Process();
    Transfo.GetTransformParam(M, cadre, mask);

    //****************************Showing Result of transformation*************************************

    std::cout << "Press SPACE to continue" << std::endl;
        
    std::string wind_stream = "Video";
    cv::namedWindow(wind_stream, cv::WINDOW_AUTOSIZE);

    cv::Mat stream;
    while(Transfo.GetView(stream))
    {
        cv::imshow(wind_stream, stream);

        if (cv::waitKey(20) == ' ')
            break;
    }
    cv::destroyWindow(wind_stream);
    stream.release();

    //****************************Select ROI for sections*************************************    
    
    Pilot Hamilton(cap, M, cadre, mask);

    Transfo.GetView(stream);
    Hamilton.SectionsSelecter(stream);

    //****************************Detection of the path by vector of points*************************************
        
    Hamilton.SavePath();
    stream.release();
    //****************************Get car separatly******************************************

    Hamilton.train(stream);
    Hamilton.drive();
   
    //****************************Quit*************************************

    std::cout << "Press any key to exit program" << std::endl;
    cv::waitKey(0);


    M.release();
    mask.release();
    cap.release();

    cv::destroyAllWindows();

    return 0;
}
