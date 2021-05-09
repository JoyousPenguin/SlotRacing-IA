#include "TrackConfig.h"


TrackConfig::TrackConfig(cv::VideoCapture cap,cv::Mat M, cv::Rect2d r, cv::Mat mask)
{
    this->tc_M = M;
    this->tc_cadre = r;
    this->tc_mask = mask;

    this->tc_cap = cap;
};


TrackConfig::~TrackConfig()
{
    tc_M.release();
    tc_mask.release();

    tc_cap.release();

    tc_StraightSections.release();
    tc_TurnSections.release();
    tc_TightTurnSections.release();
};


void TrackConfig::selection(cv::Mat& image, cv::Mat& output, int nbre)
{
    std::vector<cv::Rect2d> vec;
    cv::Mat trackMask;
    cv::resize(tc_mask, trackMask, cv::Size(), 2.0, 2.0);

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


    output = Section.clone();

    Section.release();
    image.release();
    TempSection.release();

    
};


void TrackConfig::SectionsSelecter(cv::Mat& image, cv::Mat& StraightSections, cv::Mat& TurnSections, cv::Mat& TightTurnSections)
{
    cv::Mat frame;

    //SELECT STRAIGHT SECTIONS-------------------
    frame = image.clone();
    int nbre;
    std::cout << "Enter the number of straight sections of the track : ";
    std::cin >> nbre;

    selection(frame, tc_StraightSections, nbre);
    frame.release();

    StraightSections = tc_StraightSections;

    //SELECT TIGHT TURN SECTIONS-------------------
    frame = image.clone();

    nbre = 0;
    std::cout << "Enter the number of tight turn sections of the track : ";
    std::cin >> nbre;

    selection(frame, tc_TightTurnSections, nbre);
    frame.release();

    TightTurnSections = tc_TightTurnSections;

    //SELECT TURN SECTIONS-------------------
    frame = image.clone();

    nbre = 0;
    std::cout << "Enter the number of turn sections of the track : ";
    std::cin >> nbre;

    selection(frame, tc_TurnSections, nbre);
    frame.release();

    TurnSections = tc_TurnSections;

    //DRAW ALL SECTIONS------------------------

    std::string StraightWind = "Straight sections";
    cv::namedWindow(StraightWind, cv::WINDOW_AUTOSIZE);
    imshow(StraightWind, tc_StraightSections);

    std::string TightTurntWind = "Tight turn sections";
    cv::namedWindow(TightTurntWind, cv::WINDOW_AUTOSIZE);
    imshow(TightTurntWind, tc_TightTurnSections);

    std::string TurnWind = "Turn sections";
    cv::namedWindow(TurnWind, cv::WINDOW_AUTOSIZE);
    imshow(TurnWind, tc_TurnSections);


    std::cout << "Press any Key to continue" << std::endl;
    cv::waitKey(0);

    cv::destroyWindow(StraightWind);
    cv::destroyWindow(TightTurntWind);
    cv::destroyWindow(TurnWind);
};


void TrackConfig::getPath(std::vector<cv::Point> points, cv::Mat& drawing_path)
{

    std::cout << points.size() << "points before processing | ";
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
    while (i < points.size())
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

            if (i > 50 && distTo0 < shortestDist)
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
    //cv::Mat drawing_path(img.size(), CV_8UC1, cv::Scalar(0,0,0));

    std::cout << "ordered_point_path.size() = " << ordered_point_path.size() << " --- filtered_ordered_point_path.size() = " << filtered_ordered_point_path.size() << std::endl;


    for (int i = 0; i < ordered_point_path.size(); i++)
    {
        int prev = i - 1;

        if (prev < 0)
            prev = ordered_point_path.size() + prev;

        std::cout << "Points " << i << " -- ( " << ordered_point_path[i].x << " ; " << ordered_point_path[i].y << " ) ";
        std::cout << " -- FILTERED = ( " << filtered_ordered_point_path[i].y << " ; " << filtered_ordered_point_path[i].y << " )" << std::endl;

        /*cv::circle(drawing_path, ordered_point_path[prev], 1, cv::Scalar(125), 1, cv::LINE_8);
        cv::circle(drawing_path, ordered_point_path[i], 1, cv::Scalar(125), 1, cv::LINE_8);

        line(drawing_path, ordered_point_path[prev], ordered_point_path[i], cv::Scalar(125), 1, cv::LINE_8);*/


        cv::circle(drawing_path, filtered_ordered_point_path[prev], 1, cv::Scalar(255), 1, cv::LINE_8);
        cv::circle(drawing_path, filtered_ordered_point_path[i], 1, cv::Scalar(255), 1, cv::LINE_8);

        line(drawing_path, filtered_ordered_point_path[prev], filtered_ordered_point_path[i], cv::Scalar(255), 1, cv::LINE_8);

    }
}


void TrackConfig::SavePath(cv::Mat& CarPath)
{
    //used to get the mouvement of the car
    std::vector<std::vector<cv::Point>> mvt;

    //used to locate the car at specific point
    std::vector<cv::Point> point_path;

    std::string wind = "Video";
    cv::namedWindow(wind, cv::WINDOW_AUTOSIZE);

    std::cout << "Once you have made 2 lap of the track, press Space to continue processing" << std::endl;

    cv::Mat image, stream;
    bool state = tc_cap.read(image);
    Transformation::GetView(image, stream, tc_M, tc_cadre, tc_mask);
    image.release();

    double LearningRate = 0.01;
    mvt = Detection::BackgroundSubstraction(stream, LearningRate);

    while (state)
    {
        state = tc_cap.read(image);
        Transformation::GetView(image, stream, tc_M, tc_cadre, tc_mask);
        image.release();

        mvt = Detection::BackgroundSubstraction(stream, LearningRate);


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

        if (cv::waitKey(20) == ' ')//press ESPACE
        {
            break;
        }
    }

    cv::Mat path(stream.size(), CV_8UC1, cv::Scalar(0, 0, 0));
    getPath(point_path, path);

    cv::Mat temp;

    cv::dilate(path, temp, cv::Mat(), cv::Point(-1, -1), 20);
    cv::erode(temp, CarPath, cv::Mat(), cv::Point(-1, -1), 13);

    path.release();
    temp.release();
    stream.release();

    cv::destroyWindow(wind);

};


void TrackConfig::DecomposePath(cv::Size p, std::vector<std::pair<int, int>> Track)
{
    int count = 0;
    int val = 0;
    int prevVal = 0;

    for (int i = 0; i < filtered_ordered_point_path.size(); i++)
    {
        cv::Mat Black = cv::Mat::zeros(p, CV_8U);

        cv::circle(Black, filtered_ordered_point_path[i], 1, cv::Scalar(255));

        cv::Mat result;
        val = 0;


        cv::bitwise_and(Black, Black, result, tc_StraightSections);
        if (cv::countNonZero(result) == 0)
        {
            cv::bitwise_and(Black, Black, result, tc_TurnSections);
            if (cv::countNonZero(result) == 0)
            {
                cv::bitwise_and(Black, Black, result, tc_TightTurnSections);
                if (cv::countNonZero(result) == 0)
                    val = -1;
                else
                    val = 3;
            }
            else
                val = 2;

        }
        else
            val = 1;

        if (val == prevVal || i == 0)
        {
            count++;
            prevVal = val;
        }
        else if (val == -1)
            std::cout << "Error in track" << std::endl;
        else
        {
            std::pair<int, int> answer;
            answer.first = prevVal;
            answer.second = count;

            Track.push_back(answer);

            count = 0;
            prevVal = val;
        }
    }

    for (int i = 0; i < Track.size(); i++)
    {
        std::cout << " Section: " << Track[i].first << " of size: " << Track[i].second << std::endl;
    }
};