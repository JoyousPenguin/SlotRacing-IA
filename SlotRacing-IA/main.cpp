#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>

#include <iostream>
#include <iomanip>
#include<fstream>


//vector fo point used to straighten up the image
std::vector<cv::Point> pointVec;

void mouseClick(int event, int x, int y, int, void*)
{
    cv::Point p;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN: //Left click down --> add point to vec
        p = cv::Point(x, y);
        pointVec.push_back(p);
        //std::cout << "Point added" << std::endl;
        break;
    case cv::EVENT_RBUTTONDOWN: //Right click down --> delete last point added
        if (pointVec.size() > 0)
        {
            int last = pointVec.size() - 1;
            pointVec.pop_back();
            //std::cout << "Point deleted" << std::endl;
        }
        break;
    }
}


void GetClickPoint(cv::VideoCapture cap)
{
    cv::Mat image;
    cap.read(image);
    bool mano = false;
    std::fstream Flux("data.txt", std::ios::in);
    std::vector<cv::Point> points_value;

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
            cv::Mat img = image;
            for (int i = 0; i < points_value.size(); i++)
            {
                circle(img, points_value[i], 2, cv::Scalar(0, 255, 0), 3);

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
                    //std::cout << "invalid input" << std::endl;
                    break;
                }

                cv::imshow(wind, img);
            }
            cv::destroyWindow(wind);
        }
        Flux.close();
    }

    if (mano)
    {

        image.release();
        cap.read(image);
        std::string mouseSelector = "mouse window";
        cv::namedWindow(mouseSelector);
        cv::setMouseCallback(mouseSelector, mouseClick, 0);

        std::cout << "Left click on the corner of the paper sheet" << std::endl;
        std::cout << "1---------------2" << std::endl;
        std::cout << "|               |" << std::endl;
        std::cout << "|               |" << std::endl;
        std::cout << "4---------------3" << std::endl;
        std::cout << "Right click anywere to erase last input" << std::endl;

        imshow(mouseSelector, image);

        while (pointVec.size() != 4)
        {
            if (pointVec.size() > 0)
            {

                for (int i = 0; i < pointVec.size(); i++)
                {
                    circle(image, pointVec[i], 2, cv::Scalar(0, 255, 0), 3);
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

            for (int i = 0; i < pointVec.size(); i++)
            {
                std::cout << "Point saved (" << pointVec[i].x << ", " << pointVec[i].y << ")" << std::endl;
                Flux << pointVec[i].x << std::endl;
                Flux << pointVec[i].y << std::endl;
            }

            std::cout << "data saved" << std::endl;
            Flux.close();
        }
    }
    else
        pointVec = points_value;
}


cv::Mat GetTransformMat(cv::Mat frame, std::vector<cv::Point> element)
{
    //will get the transform matrix to apply to get a straight track

    if (element.size() == 4)
    {
        std::cout << "TRANFORM" << std::endl;

        cv::Point2f tab_point_src[4];
        int mid_height = frame.size().height / 2;
        int mid_width = frame.size().width / 2;

        cv::Point2f tab_point_dst[4];

        cv::Mat M, transform;
        transform.size() = frame.size();

        //ratio 20/30 --> feuille A4
        for (int i = 0; i < element.size(); i++)
        {
            //a patir de la position des brique on rempli le tableau src
            tab_point_src[i] = element[i];
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
        M = cv::getPerspectiveTransform(tab_point_src, tab_point_dst);

        //on applique la matrice de transfo 3x3 a toute l'image 
        cv::warpPerspective(frame, transform, M, transform.size());

        transform.release();

        return M;
        M.release();
    }
    else
    {
        std::cout << "Too many elements" << std::endl;
        cv::Mat Null;
        return Null;
        Null.release();
    }


}


void GetTransformViewParam(cv::Mat& image, cv::Mat& M, cv::Rect2d& finalRect, cv::Mat& finalMask)
{
    //****************************Computing of the transformation matrix********************

    M = GetTransformMat(image, pointVec);

    cv::Mat start;
    start.size() = image.size();
    cv::warpPerspective(image, start, M, image.size());
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
        cv::warpPerspective(blank, view_blank, M, blank.size());
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
        finalRect = boundingRect(FirstHull[largest_contour_index]);

        finalRect.width += offset_rect;
        finalRect.height += offset_rect;
        finalRect.x -= offset_rect / 2;
        finalRect.y -= offset_rect / 2;


        if (finalRect.x < 0)
            finalRect.x = 0;
        if (finalRect.y < 0)
            finalRect.y = 0;
        if (finalRect.y + finalRect.height > view.rows)
            finalRect.height -= (finalRect.y + finalRect.height - view.rows);
        if (finalRect.x + finalRect.width > view.cols)
            finalRect.width -= (finalRect.x + finalRect.width - view.cols);

        cv::Mat im(view);
        cv::Mat finalView = im(finalRect);

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

        finalMask = cv::Mat::zeros(finalView.size(), CV_8U);
        for (int i = 0; i < Hull_mask.size(); i++)
        {
            //xor all the hull mat so the center of thetrack is not in the mask
            bitwise_xor(finalMask, Hull_mask[i], finalMask, cv::noArray());
        }

        cv::dilate(finalMask, finalMask, cv::Mat(), cv::Point(-1, -1), 2);

        imshow(wind_result, finalMask);

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

    


}


void TransformView(cv::Mat& image, cv::Mat& output, cv::Mat& M, cv::Rect2d& r, cv::Mat& mask)
{
    cv::Mat flat, bitAndImg, bitAndMask;
    cv::warpPerspective(image, flat, M, image.size());

    cv::Mat im(flat);
    cv::Mat resized = im(r);

    bitwise_and(resized, resized, bitAndImg, mask);

    cv::resize(bitAndImg, output, cv::Size(), 2, 2, cv::INTER_LINEAR);

    flat.release();
    resized.release();
}


cv::Ptr<cv::BackgroundSubtractor> initBackSub()
{
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

    int history = 500;      //how many last frames affect the background model.
    double varTreshold = 400;
    bool detectShadow = false;

    pBackSub = cv::createBackgroundSubtractorMOG2(history, varTreshold, detectShadow);

    return pBackSub;
}


std::vector<std::vector<cv::Point>> BackgroundSubstraction(cv::Ptr<cv::BackgroundSubtractor> BackSub, cv::Mat& stream)
{
    double learningRate = 0.01;


    cv::Mat fgmask;
    //fgmask.size() = image.size();
    BackSub->apply(stream, fgmask, learningRate);

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

    fgmask.release();
    fgmask_tresh.release();
    dilated.release();
    eroded.release();


    return Edges;
    
}


void getPath(std::vector<cv::Point> points)
{

    std::cout << points.size()<<"points before processing | ";
    //erase point that are the same
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = i + 1; j < points.size(); j++)
        {
            if (i != j && points[i] == points[j])
            {
                points.erase(points.begin() + j);
                j--;
            }
        }
    }

    std::cout << points.size() << " points after processing" << std::endl;



    //get the points in the right order  
    std::vector<cv::Point> ordered_point_path;
    std::vector<int> usedIdx;

    ordered_point_path.push_back(points[0]);
    usedIdx.push_back(0);

    
    int shortestDistIdx = 0;
    int presIdx = 0;
    bool used = false;

    int i = 0;
    while(i < points.size())
    {
        double shortestDist = 9999999;
        if (i == points.size() - 1)
        {
            shortestDistIdx = 0;
        }
        else
        {
            for (int j = 0; j < points.size(); j++)
            {
                if (presIdx != j)
                {
                    double dist = sqrt(pow(points[presIdx].x - points[j].x, 2) + pow(points[presIdx].y - points[j].y, 2));
                    used = false;
                    if (dist < shortestDist)
                    {
                        for (int k = 0; k < usedIdx.size(); k++)
                        {
                            if (usedIdx[k] == j)
                                used = true;
                        }
                        if (!used)
                        {
                            shortestDist = dist;
                            shortestDistIdx = j;
                        }
                    }
                }
            }

            double distTo0 = sqrt(pow(points[presIdx].x - points[0].x, 2) + pow(points[presIdx].y - points[0].y, 2));

            if(i> 50 && distTo0 < shortestDist)
            {
                shortestDistIdx = 0;
            }
        }
        ordered_point_path.push_back(points[shortestDistIdx]);
        usedIdx.push_back(shortestDistIdx);
        presIdx = shortestDistIdx;
        i++;

        if (shortestDistIdx == 0)
            break;
    }


    //filter the points
    //get the average of the point before and after the present point

    std::vector<cv::Point2f> filtered_ordered_point_path;


    for (int i = 0; i < ordered_point_path.size(); i++)
    {
        int prev = i - 1;
        int fut = i + 1;

        if (prev < 0)
            prev = ordered_point_path.size() - 1;
        if (fut > ordered_point_path.size() - 1)
            fut = 0;
        
        cv::Point2f p;

        p.x = (ordered_point_path[prev].x + ordered_point_path[i].x + ordered_point_path[fut].x) / 3;
        p.y = (ordered_point_path[prev].y + ordered_point_path[i].y + ordered_point_path[fut].y) / 3;

        filtered_ordered_point_path.push_back(p);
    }


    //Draw the points of the track
    cv::Mat drawing_path(cv::Size(400, 500), CV_8UC1, cv::Scalar(0));

    std::cout << "ordered_point_path.size() = " << ordered_point_path.size() << " --- filtered_ordered_point_path.size() = "<< filtered_ordered_point_path.size() << std::endl;


    for (int i = 0; i < ordered_point_path.size(); i++)
    {
        int prev = i - 1;

        if (prev < 0)
            prev = ordered_point_path.size() + prev;

        std::cout << "Points " << i << " -- ( " << ordered_point_path[i].x << " ; " << ordered_point_path[i].y << " ) ";
        std::cout << " -- FILTERED = ( " << filtered_ordered_point_path[i].y << " ; " << filtered_ordered_point_path[i].y << " )" << std::endl;

        cv::circle(drawing_path, ordered_point_path[prev], 1, cv::Scalar(125), 1, cv::LINE_8);
        cv::circle(drawing_path, ordered_point_path[i], 1, cv::Scalar(125), 1, cv::LINE_8);

        line(drawing_path, ordered_point_path[prev], ordered_point_path[i], cv::Scalar(125), 1, cv::LINE_8);


        cv::circle(drawing_path, filtered_ordered_point_path[prev], 1, cv::Scalar(255), 1, cv::LINE_8);
        cv::circle(drawing_path, filtered_ordered_point_path[i], 1, cv::Scalar(255), 1, cv::LINE_8);

        line(drawing_path, filtered_ordered_point_path[prev], filtered_ordered_point_path[i], cv::Scalar(255), 1, cv::LINE_8);

        cv::imshow("demo", drawing_path);
    }



    
    








    //computing Slope of point of the track
    //if slope is infinite --> replaced by previous value
    std::vector<double> slope_path_point;
    std::vector<cv::Point> StraightLine;

    int shift_point = 5;
    int shift_pixel = 3;
    for (int i = 0; i < filtered_ordered_point_path.size(); i++)
    {
        double pos, neg;

        if ((i + shift_point) > filtered_ordered_point_path.size() - 1)
            pos = (i + shift_point) - filtered_ordered_point_path.size();

        else
            pos = i + shift_point;

        if ((i - shift_point) < 0)
            neg = filtered_ordered_point_path.size() + (i - shift_point);
        else
            neg = i - shift_point;



        double DeltaX = filtered_ordered_point_path[pos].x - filtered_ordered_point_path[neg].x;
        double Deltay = filtered_ordered_point_path[pos].y - filtered_ordered_point_path[neg].y;


        if (DeltaX < shift_pixel || Deltay < shift_pixel)
        {
            StraightLine.push_back(filtered_ordered_point_path[i]);
        }
        else 
        {
            StraightLine.push_back(cv::Point(-1, -1));
        }

        double slope = abs(Deltay / DeltaX);

        slope_path_point.push_back(slope);
    }

    cv::Mat drawing_straight_path(cv::Size(400, 500), CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < StraightLine.size(); i++)
    {
        if (StraightLine[i] != cv::Point(-1, -1))
        {
            cv::circle(drawing_straight_path, StraightLine[i], 1, cv::Scalar(255), 1, cv::LINE_8);
        }
    }


    cv::imshow("Line -Delta", drawing_straight_path);

    cv::waitKey(0);

    //apply a LINEAR FILTER TO THE VALUES OF SLOPES
    /*std::vector<double> filtered_slope_path_point;

   

    for (int i = 0; i < slope_path_point.size(); i++)
    {
        //std::cout << "i = " << i << std::endl;

        int prev = i - 1;
        if (prev < 0)
            prev = slope_path_point.size() - 1;

        int fut = i + 1;
        if (fut > slope_path_point.size() - 1)
            fut = 0;

        //std::cout << " -- prev = " << prev << " -- fut = " << fut << std::endl;

        if (slope_path_point[prev] == slope_path_point[fut] == slope_path_point[i]) //case X X X
        {
            //std::cout << "COUCOU 1" << std::endl;
            filtered_slope_path_point.push_back(slope_path_point[i]);
        }
        else
        {
            //std::cout << "COUCOU 2" << std::endl;
            if (slope_path_point[prev] != slope_path_point[i] && slope_path_point[fut] != slope_path_point[i] && slope_path_point[prev] == slope_path_point[fut]) //case X Y X 
            {
                //std::cout << "COUCOU 2.1" << std::endl;
                filtered_slope_path_point.push_back(slope_path_point[prev]);
            }
            else if(slope_path_point[prev] != slope_path_point[i] && slope_path_point[i]==slope_path_point[fut]) // case Y X X 
            {
                filtered_slope_path_point.push_back(slope_path_point[i]);

            }
            else if (slope_path_point[prev] == slope_path_point[i] && slope_path_point[i] != slope_path_point[fut]) //case X X Y
            {
                filtered_slope_path_point.push_back(slope_path_point[i]);

            }
            else if (slope_path_point[prev] != slope_path_point[i] != slope_path_point[fut]) // case X Y Z
            {
                //std::cout << "COUCOU 2.2" << std::endl;

                int prev2 = i - 2;
                if (prev2 < 0)
                    prev2 = slope_path_point.size() + prev2;

                int fut2 = i + 2;
                if (fut2 > slope_path_point.size() - 1)
                    fut2 = fut2 - slope_path_point.size();

                //std::cout <<" -- prev2 = " <<prev2<<" -- fut2 = "<<fut2<< std::endl;

                if (slope_path_point[prev2] == slope_path_point[prev] && slope_path_point[fut] == slope_path_point[fut2]) //case X X Y Z Z
                {
                    //std::cout << "COUCOU 2.2.1" << std::endl;
                    filtered_slope_path_point.push_back(slope_path_point[prev]);
                }
                else
                {
                    //std::cout << "COUCOU 2.2.2" << std::endl;
                    filtered_slope_path_point.push_back(slope_path_point[i]);
                }
            }
            else //case X X Y
            {
                filtered_slope_path_point.push_back(slope_path_point[i]);
            }
        }
        std::cout << slope_path_point[i] << " ---- " << filtered_slope_path_point[i] << std::endl;
    }*/




    //check where in the track the slope is +/- constant
        
    //std::vector<std::vector<int>> straightLine;
    std::vector<int> temp;

    double offsetVal = 0.2;
    /*for (int i = 0; i < filtered_slope_path_point.size(); i++)
    {
        bool stop = false;
        int Idx = 0;

        while (!stop)
        {
            int NextIdx = i + Idx;


            if (NextIdx > filtered_slope_path_point.size() - 1)
            {
                NextIdx = NextIdx - (filtered_slope_path_point.size());
            }              

                

            if (filtered_slope_path_point[NextIdx] == filtered_slope_path_point[i] && (filtered_slope_path_point[i]==3 || filtered_slope_path_point[i] == 0.1))
                temp.push_back(NextIdx);
            else
                stop = true;

            Idx++;

            if (Idx == filtered_slope_path_point.size() - 2)
                break;

        }
        //std::cout << "COUCOU - "<<i << std::endl;
        straightLine.push_back(temp);
        temp.clear();
    }*/
   
    int next = 1;
    for (int i = 0; i < filtered_ordered_point_path.size(); i++)
    {
        int pos, neg;

        if ((i + next) > filtered_ordered_point_path.size() - 1)
            pos = (i + next) - filtered_ordered_point_path.size();
        else
            pos = i + next;


        if ((i - next) < 0)
            neg = filtered_ordered_point_path.size() + (i - next);
        else
            neg = i - next;



        if (slope_path_point[i] < (1 + offsetVal) * slope_path_point[neg] && slope_path_point[i] > (1 - offsetVal) * slope_path_point[neg] &&
            slope_path_point[i] < (1 + offsetVal) * slope_path_point[pos] && slope_path_point[i] > (1 - offsetVal) * slope_path_point[pos])
        {
            if (StraightLine[i] != cv::Point(-1, -1))
            {
                StraightLine[i] = filtered_ordered_point_path[i];
            }
        }

    }

    


    /*for (int i = 0; i < straightLine.size(); i++)
    {
        if (straightLine[i].size() > 2)
        {
            for (int j = 1; j < straightLine[i].size(); j++)
            {
                cv::circle(drawing_straight_path, filtered_ordered_point_path[straightLine[i][j - 1]], 1, cv::Scalar(255), 1, cv::LINE_8);
                cv::circle(drawing_straight_path, filtered_ordered_point_path[straightLine[i][j]], 1, cv::Scalar(255), 1, cv::LINE_8);


                cv::line(drawing_straight_path, filtered_ordered_point_path[straightLine[i][j - 1]], filtered_ordered_point_path[straightLine[i][j]], cv::Scalar(255), 1, cv::LINE_8);
            }
        }
    }*/

    for (int i = 0; i < StraightLine.size(); i++)
    {
        if (StraightLine[i] != cv::Point(-1, -1))
        {
            cv::circle(drawing_straight_path, StraightLine[i], 1, cv::Scalar(255), 1, cv::LINE_8);
        }
    }


    cv::imshow("Line", drawing_straight_path);
    


}


cv::Mat selection(cv::Mat image, cv::Mat mask,  int nbre)
{

    std::vector<cv::Rect2d> vec;

    cv::Mat trackMask;
    cv::resize(mask, trackMask, cv::Size(), 2.0, 2.0);

    std::string wind_roi = "Select sections";
    cv::namedWindow(wind_roi, cv::WINDOW_AUTOSIZE);


    bool fromCenter = false;


    while (vec.size() < nbre)
    {
        char car = cv::waitKey(10);

        cv::Rect2d r = cv::selectROI(wind_roi, image, fromCenter);

        cv::rectangle(image, r, cv::Scalar(0, 255, 0), 2);

        vec.push_back(r);
    }


    cv::Mat TempSection = cv::Mat::zeros(image.size(), CV_8U);
    cv::Mat Section = cv::Mat::zeros(image.size(), CV_8U);

    for (int i = 0; i < vec.size(); i++)
    {
        cv::rectangle(TempSection, vec[i], cv::Scalar(255), -1);
    }

    cv::destroyWindow(wind_roi);

    cv::bitwise_and(TempSection, trackMask, Section, cv::Mat());

    image.release();
    TempSection.release();

    return Section;
    Section.release();
}


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
    cv::destroyWindow(window_start);

    //****************************Starting process to isolate track*************************************

    //get the points to straighten up th eimage
    GetClickPoint(cap);

    cv::Mat M, mask;
    cv::Rect2d cadre;
    GetTransformViewParam(image, M, cadre, mask);

    cv::Mat fixed_view;
    TransformView(image, fixed_view, M, cadre, mask);

    //****************************Showing Result of transforation*************************************

    std::cout << "Press SPACE to continue" << std::endl;

    cv::Mat video_stream;
    cv::Mat stream;
    std::string wind_stream = "Video";
    cv::namedWindow(wind_stream, cv::WINDOW_AUTOSIZE);


    while(state)
    {
        state=cap.read(image);

        TransformView(image, stream, M, cadre, mask);
        //cv::resize(video_stream, stream, cv::Size(), 2, 2, cv::INTER_LINEAR);
        cv::imshow(wind_stream, stream);

        if (cv::waitKey(20) == ' ')
            break;

    }
    cv::destroyWindow(wind_stream);
    video_stream.release();

    //****************************Select ROI for sections*************************************

    
    
    
    //SELECT STRAIGHT SECTIONS-------------------
    image.release();
    cap.read(image);   

    cv::Mat selected;
    TransformView(image, selected, M, cadre, mask);

    int nbre;
    std::cout << "Enter the number of straight sections of the track : ";
    std::cin >> nbre;

    cv::Mat StraightSection = selection(selected, mask, nbre);

    selected.release();

    //SELECT TIGHT TURN SECTIONS-------------------
    image.release();
    cap.read(image);

    TransformView(image, selected, M, cadre, mask);

    nbre = 0;
    std::cout << "Enter the number of tight turn sections of the track : ";
    std::cin >> nbre;

    cv::Mat TightTurnSection = selection(selected, mask, nbre);

    selected.release();

    //SELECT TURN SECTIONS-------------------
    image.release();
    cap.read(image);

    TransformView(image, selected, M, cadre, mask);

    nbre = 0;
    std::cout << "Enter the number of turn sections of the track : ";
    std::cin >> nbre;

    cv::Mat TurnSection = selection(selected, mask, nbre);

    selected.release();

    //DRAW ALL SECTIONS------------------------

    std::string StraightWind = "Straight sections";
    cv::namedWindow(StraightWind, cv::WINDOW_AUTOSIZE);
    imshow(StraightWind, StraightSection);

    std::string TightTurntWind = "Tight turn sections";
    cv::namedWindow(TightTurntWind, cv::WINDOW_AUTOSIZE);
    imshow(TightTurntWind, TightTurnSection);

    std::string TurnWind = "Turn sections";
    cv::namedWindow(TurnWind, cv::WINDOW_AUTOSIZE);
    imshow(TurnWind, TurnSection);

    cv::waitKey(1);









    //****************************Detection of the path*************************************

    //initialize background substraction
    cv::Ptr<cv::BackgroundSubtractor> BackSub = initBackSub();
    std::vector<std::vector<cv::Point>> mvt;

    std::vector<cv::Point> point_path;

    

    std::string wind = "Video";
    cv::namedWindow(wind, cv::WINDOW_AUTOSIZE);

    std::cout << "Once you have made 2 lap of the track, press Space to continue processing" << std::endl;

    

    state = cap.read(image);
    TransformView(image, stream, M, cadre, mask);
    //cv::resize(video_stream, stream, cv::Size(), 2, 2, cv::INTER_LINEAR);
    mvt = BackgroundSubstraction(BackSub, stream);

    while (state)
    {
        cv::Mat video_stream, stream;

        state=cap.read(image);

        TransformView(image, stream, M, cadre, mask);

        //cv::resize(video_stream, stream, cv::Size(), 2,2, cv::INTER_LINEAR);

        mvt = BackgroundSubstraction(BackSub, stream);

        if (mvt.size() <= 2)
        {
            for (int i = 0; i < mvt.size(); i++)
            {
                cv::Rect2d r = boundingRect(mvt[i]);
                rectangle(stream, r, cv::Scalar(0, 255, 0), 2);

                cv::Point p;

                p.x = r.x + r.width / 2;
                p.y = r.y + r.height / 2;

                point_path.push_back(p);
            }
        }
        
        imshow(wind, stream);
        video_stream.release();
        stream.release();

        if (cv::waitKey(20) == ' ')//press ESPACE
        {
            break;
        }
        
    }

    getPath(point_path);


    //****************************Quit*************************************

    std::cout << "Press any key to exit program" << std::endl;
    cv::waitKey(0);

    image.release();
    M.release();
    mask.release();

    cv::destroyAllWindows();
    cap.release();

    return 0;
}
