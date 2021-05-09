#include "Transformation.h"

Transformation::Transformation(cv::VideoCapture cap)
{
    this->t_cap = cap;
};


Transformation::Transformation()
{ };


Transformation::~Transformation()
{
    t_cap.release();

    t_M.release();
    t_mask.release();
};


void mouseClick(int event, int x, int y, int, void* param)
{

    std::vector<cv::Point>* vec = (std::vector<cv::Point>*)param;
    cv::Point p;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN: //Left click down --> add point to vec
        p = cv::Point(x, y);
        vec->push_back(p);
        //std::cout << "Point added" << std::endl;
        break;
    case cv::EVENT_RBUTTONDOWN: //Right click down --> delete last point added
        if (vec->size() > 0)
        {
            int last = vec->size() - 1;
            vec->pop_back();
            //std::cout << "Point deleted" << std::endl;
        }
        break;
    }
};


void Transformation::GetClickPoint()
{
    cv::Mat image;

    //used to know with user will enter new points or used the old one
    bool mano = false;

    //open file
    std::fstream Flux("data.txt", std::ios::in);

    //vector used to store the points value before storing them in pointVec global vector
    std::vector<cv::Point> points_value;

    t_cap.read(image);

    if (Flux)
    {
        std::cout << "fichier ouvert" << std::endl;
        std::string ligne;
        getline(Flux, ligne);

        if (ligne == "")
        {
            std::cout << "no data in data.txt -- " << std::endl;
            mano = true;
        }
        else if (ligne != "")
        {
            cv::Point2f p;
            int i = 0;
            while (ligne != "")
            {
                if (i == 0)
                {
                    p.x = stoi(ligne);
                    i++;
                }
                else
                {
                    p.y = stoi(ligne);
                    points_value.push_back(p);
                    i = 0;
                }
                getline(Flux, ligne);
            }

            for (int i = 0; i < points_value.size(); i++)
            {
                cv::circle(image, points_value[i], 2, cv::Scalar(0, 255, 0), 3);
            }

            std::cout << "press SPACE to accpet points" << std::endl;
            std::cout << "press ESC to delete points" << std::endl;

            char car;
            bool choice = false;

            std::string wind = "Select Point";
            cv::namedWindow(wind, cv::WINDOW_AUTOSIZE);

            while (!choice)
            {
                car = cv::waitKey(50);

                switch (car)
                {
                case' ':
                    mano = false;
                    choice = true;
                    break;
                case 27:
                    mano = true;
                    choice = true;
                    break;
                default:
                    break;
                }

                cv::imshow(wind, image);
                
            }
            cv::destroyWindow(wind);
        }
        Flux.close();
        image.release();
    }

    if (mano)
    {

        t_cap.read(image);
        std::string mouseSelector = "mouse window";
        cv::namedWindow(mouseSelector);
        cv::setMouseCallback(mouseSelector, mouseClick, &t_pointVec);

        std::cout << "Left click on the corner of the paper sheet" << std::endl;
        std::cout << "1---------------2" << std::endl;
        std::cout << "|               |" << std::endl;
        std::cout << "|               |" << std::endl;
        std::cout << "4---------------3" << std::endl;
        std::cout << "Right click anywere to erase last input" << std::endl;

        imshow(mouseSelector, image);

        while (t_pointVec.size() != 4)
        {
            if (t_pointVec.size() > 0)
            {

                for (int i = 0; i < t_pointVec.size(); i++)
                {
                    circle(image, t_pointVec[i], 2, cv::Scalar(0, 255, 0), 3);
                }
            }
            imshow(mouseSelector, image);
            cv::waitKey(10);
        }

        cv::destroyWindow(mouseSelector);
        std::cout << "Ref found" << std::endl;

        std::fstream Flux("data.txt", std::ios::out);
        if (Flux)
        {
            std::cout << "fichier ouvert" << std::endl;
            Flux.clear();
            Flux.seekg(0, std::ios::beg);

            //Save point in data.txt file
            for (int i = 0; i < t_pointVec.size(); i++)
            {
                std::cout << "Point saved (" << t_pointVec[i].x << ", " << t_pointVec[i].y << ")" << std::endl;
                Flux << t_pointVec[i].x << std::endl;
                Flux << t_pointVec[i].y << std::endl;
            }

            std::cout << "data saved" << std::endl;
            Flux.close();
        }
    }
    else
        t_pointVec = points_value;
};


void Transformation::GetTransformMat(cv::Mat& frame)
{
    //will get the transform matrix to apply to get a straight track
    std::cout << "TRANFORM" << std::endl;

    cv::Point2f tab_point_src[4];
    int mid_height = frame.size().height / 2;
    int mid_width = frame.size().width / 2;

    cv::Point2f tab_point_dst[4];

    cv::Mat transform;
    transform.size() = frame.size();

    //ratio 20/30 --> A4 paper sheet
    for (int i = 0; i < t_pointVec.size(); i++)
    {
        //a patir de la position des brique on rempli le tableau src
        tab_point_src[i] = t_pointVec[i];
    }

    tab_point_dst[0].x = mid_height - 30;
    tab_point_dst[0].y = mid_width + 20;

    tab_point_dst[1].x = mid_height + 30;
    tab_point_dst[1].y = mid_width + 20;

    tab_point_dst[2].x = mid_height + 30;
    tab_point_dst[2].y = mid_width - 20;

    tab_point_dst[3].x = mid_height - 30;
    tab_point_dst[3].y = mid_width - 20;


    //by comparing the src and dst of the position of the corner of sheet of paper we obtain a 3x3 transformation matrix
    t_M = cv::getPerspectiveTransform(tab_point_src, tab_point_dst);

    //on applique la matrice de transfo 3x3 a toute l'image 
    cv::warpPerspective(frame, transform, t_M, transform.size());

    transform.release();

};


void Transformation::ComputeTransformViewParam()
{

    cv::Mat image;
    t_cap.read(image);

    //****************************Computing of the transformation matrix********************

    GetTransformMat(image);

    cv::Mat start;
    start.size() = image.size();
    cv::warpPerspective(image, start, t_M, image.size());
    //view = straight image with the track but very little


    //****************************Canny edges detection************************************


    //user can modify contrast and brightness to obtain the best mask possible

    double alpha = 1.0; //var for contrast
    double beta = 0;    //var for brightness

    char car;
    bool ok = false;

    std::cout << "7 & 4 -- contrast   [1.0-3.0]" << std::endl;
    std::cout << "9 & 6 -- brightness [0 - 100]" << std::endl;

    cv::Mat view; //use to show the image with the different contrast and brightness parameters

    std::string wind_before = "Before";
    cv::namedWindow(wind_before, cv::WINDOW_AUTOSIZE);

    std::string wind_after = "After";
    cv::namedWindow(wind_after, cv::WINDOW_AUTOSIZE);

    std::string wind_result = "Result";
    cv::namedWindow(wind_result, cv::WINDOW_AUTOSIZE);

    while (!ok)
    {
        car = cv::waitKey(20);

        switch (car)
        {
        case '7':

            alpha += 0.1;
            if (alpha > 3)
                alpha = 3.0;
            std::cout << "alpha = " << alpha << std::endl;
            break;
        case'4':
            alpha -= 0.1;
            if (alpha < 1.0)
                alpha = 1.0;
            std::cout << "alpha = " << alpha << std::endl;
            break;
        case '9':
            beta += 0.1;
            if (beta > 100)
                beta = 100;
            std::cout << "beta = " << beta << std::endl;
            break;
        case'6':
            beta -= 0.1;
            if (beta < 0)
                beta = 0;
            std::cout << "beta = " << beta << std::endl;
            break;
        case ' ':
            ok = true;
            std::cout << "Validated" << std::endl;
            break;
        default:
            break;
        }

        start.convertTo(view, -1, alpha, beta);


        cv::imshow(wind_before, start);
        cv::imshow(wind_after, view);


        //First canny edge process
       //=============================

        const int HighThreshold = 200;
        const int LowThreshold = 150;
        const int offset_rect = 4;
        const int minAreaSurface = 100;

        std::vector<std::vector<cv::Point>> FirstContours;

        cv::Mat gray;
        cv::cvtColor(view, gray, cv::COLOR_RGBA2GRAY, 0);

        cv::Mat blured;
        cv::GaussianBlur(gray, blured, cv::Size(3, 3), 0, 0, 0);

        cv::Mat FirstEdges;
        cv::Canny(blured, FirstEdges, LowThreshold, HighThreshold, 3, true);


        cv::Mat view_blank;
        //create a mask equivalent to transformation but eroded
        //so the side of the image aren't suppose to be detected by the canny edges detection
        cv::Mat blank(image.size(), CV_8UC1, cv::Scalar(255));
        const int erode_size = 2;
        cv::warpPerspective(blank, view_blank, t_M, blank.size());
        erode(view_blank, view_blank, cv::Mat(), cv::Point(-1, -1), erode_size, 0);


        cv::Mat newFirstEdges;
        //filter the size of the image ---> new edges = resultat of canny without the detection on the side of the image
        cv::bitwise_and(FirstEdges, FirstEdges, newFirstEdges, view_blank);

        //look for the contours to get a zoom of the track
        cv::findContours(newFirstEdges, FirstContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        std::vector<std::vector<cv::Point> > FirstHull(FirstContours.size());
        cv::Mat mask = cv::Mat::zeros(view.size(), CV_8U);

        for (int i = 0; i < FirstContours.size(); i++)
        {
            convexHull(cv::Mat(FirstContours[i]), FirstHull[i], false);

            if (arcLength(FirstContours[i], true) > 150)
            {
                drawContours(mask, FirstHull, i, cv::Scalar(255), -1, 8); //thickness == -1 --> draw inside contours
            }
        }
        //mask = black image with white track


        cv::Mat newView;
        cv::bitwise_and(view, view, newView, mask); //newView = view of the camera with the filter of the track thanks to mask

        //get the biggest contours
        int largest_area = 0;
        int largest_contour_index = 0;

        for (size_t i = 0; i < FirstHull.size(); i++) // iterate through each contour.
        {
            double area = contourArea(FirstHull[i]);  //  Find the area of contour

            if (area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;               //Store the index of largest contour
            }
        }


        //reframe teh image so there is only the track remaining
        t_cadre = boundingRect(FirstHull[largest_contour_index]);

        t_cadre.width += offset_rect;
        t_cadre.height += offset_rect;
        t_cadre.x -= offset_rect / 2;
        t_cadre.y -= offset_rect / 2;


        if (t_cadre.x < 0)
            t_cadre.x = 0;
        if (t_cadre.y < 0)
            t_cadre.y = 0;
        if (t_cadre.y + t_cadre.height > view.rows)
            t_cadre.height -= (t_cadre.y + t_cadre.height - view.rows);
        if (t_cadre.x + t_cadre.width > view.cols)
            t_cadre.width -= (t_cadre.x + t_cadre.width - view.cols);

        cv::Mat im(view);
        cv::Mat finalView = im(t_cadre);

        //Second canny edge process
       //=============================

        cv::Mat gray_finalView;
        cv::cvtColor(finalView, gray_finalView, cv::COLOR_RGBA2GRAY, 0);


        cv::Mat blured_finalView;
        cv::GaussianBlur(gray_finalView, blured_finalView, cv::Size(3, 3), 0, 0, 0);

        cv::Mat SecondEdges;
        cv::Canny(blured_finalView, SecondEdges, LowThreshold, HighThreshold, 3, true);

        //create a matrix with a white rect so canny edges detection do not consider the side oh the image
        cv::Mat SecondBlank(SecondEdges.size(), CV_8UC1, cv::Scalar(0));
        cv::rectangle(SecondBlank, cv::Point(2, 2), cv::Point(SecondBlank.cols - 2, SecondBlank.rows - 2), cv::Scalar(255), -1);

        cv::Mat newSecondEdges;
        cv::bitwise_and(SecondEdges, SecondEdges, newSecondEdges, SecondBlank);


        std::vector<std::vector<cv::Point>> SecondContours;
        cv::findContours(newSecondEdges, SecondContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        std::vector<std::vector<cv::Point>> SecondHull;
        std::vector<cv::Mat> Hull_mask;

        for (int i = 0; i < SecondContours.size(); i++)
        {
            std::vector<cv::Point> HULL;

            convexHull(SecondContours[i], HULL, false);

            if (arcLength(SecondContours[i], true) > minAreaSurface)
            {
                SecondHull.push_back(HULL);
                Hull_mask.push_back(cv::Mat::zeros(finalView.size(), CV_8U));
            }
        }

        //draw on one mat, one hull
        for (int j = 0; j < Hull_mask.size(); j++)
        {
            drawContours(Hull_mask[j], SecondHull, j, cv::Scalar(255), -1, cv::LINE_8);
        }

        t_mask = cv::Mat::zeros(finalView.size(), CV_8U);
        for (int i = 0; i < Hull_mask.size(); i++)
        {
            //xor all the hull mat so the center of thetrack is not in the mask
            bitwise_xor(t_mask, Hull_mask[i], t_mask, cv::noArray());
        }

        cv::dilate(t_mask, t_mask, cv::Mat(), cv::Point(-1, -1), 2);

        imshow(wind_result, t_mask);

        gray.release();
        blured.release();
        FirstEdges.release();

        view_blank.release();
        blank.release();

        newFirstEdges.release();
        mask.release();

        newView.release();
        finalView.release();

        gray_finalView.release();
        blured_finalView.release();
        SecondEdges.release();

        SecondBlank.release();
        newSecondEdges.release();

    }
    std::cout << "Transform Param transfered" << std::endl;

    cv::destroyWindow(wind_result);
    cv::destroyWindow(wind_before);
    cv::destroyWindow(wind_after);

    start.release();
    view.release();
};


void Transformation::Process()
{
    GetClickPoint();

    ComputeTransformViewParam();

};


bool Transformation::GetView(cv::Mat& output)
{
    cv::Mat image, flat, bitAndImg;

    bool state = t_cap.read(image);
    
    if (state)
    {
        cv::warpPerspective(image, flat, t_M, image.size());

        cv::Mat im(flat);
        cv::Mat resized = im(t_cadre);

        bitwise_and(resized, resized, bitAndImg, t_mask);

        cv::resize(bitAndImg, output, cv::Size(), 2, 2, cv::INTER_LINEAR);

        resized.release();
    }    

    flat.release();
    bitAndImg.release();

    return state;
};


void Transformation::GetView(cv::Mat& input, cv::Mat& output, cv::Mat& M, cv::Rect2d& r, cv::Mat& mask)
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


void Transformation::GetTransformParam(cv::Mat& M, cv::Rect2d& r, cv::Mat& mask)
{
    M = this->t_M;
    r = this->t_cadre;
    mask = this->t_mask;
};