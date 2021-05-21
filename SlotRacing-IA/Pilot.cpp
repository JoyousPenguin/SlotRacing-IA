#include "Pilot.h"

Pilot::Pilot(cv::VideoCapture cap, cv::Mat M, cv::Rect2d r, cv::Mat mask)
{
    this->p_M = M;
    this->p_cadre = r;
    this->p_mask = mask;

    this->p_cap = cap;

    //connect to Arduino
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


void Pilot::selection(cv::Mat& image, cv::Mat& output, int nbre)
{
    std::vector<cv::Rect2d> vec;
    cv::Mat trackMask;
    cv::resize(p_mask, trackMask, cv::Size(), 2.0, 2.0);

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


void Pilot::SectionsSelecter(cv::Mat& image)
{
    cv::Mat frame;

    //SELECT STRAIGHT SECTIONS-------------------
    frame = image.clone();
    int nbre;
    std::cout << "Enter the number of straight sections of the track : ";
    std::cin >> nbre;

    selection(frame, StraightSections, nbre);
    frame.release();

    //SELECT TIGHT TURN SECTIONS-------------------
    frame = image.clone();

    nbre = 0;
    std::cout << "Enter the number of tight turn sections of the track : ";
    std::cin >> nbre;

    selection(frame, TightTurnSections, nbre);
    frame.release();

    //SELECT TURN SECTIONS-------------------
    frame = image.clone();

    nbre = 0;
    std::cout << "Enter the number of turn sections of the track : ";
    std::cin >> nbre;

    selection(frame, TurnSections, nbre);
    frame.release();

    //SELECT START GRID
    frame = image.clone();
    std::cout << "Select the start grid of the track" << std::endl;
    nbre = 1;
    std::string wind_roi = "Select sections";
    cv::namedWindow(wind_roi, cv::WINDOW_AUTOSIZE);

    Startgrid= cv::selectROI(wind_roi, frame, false);
    std::cout << "Startgrid position to (" << Startgrid.x << ";" << Startgrid.y << ") with dim " << Startgrid.width << " x " << Startgrid.height << std::endl;
    cv::destroyWindow(wind_roi);
    frame.release();

    //DRAW ALL SECTIONS------------------------

    std::string StraightWind = "Straight sections";
    cv::namedWindow(StraightWind, cv::WINDOW_AUTOSIZE);
    imshow(StraightWind, StraightSections);

    std::string TightTurntWind = "Tight turn sections";
    cv::namedWindow(TightTurntWind, cv::WINDOW_AUTOSIZE);
    imshow(TightTurntWind, TightTurnSections);

    std::string TurnWind = "Turn sections";
    cv::namedWindow(TurnWind, cv::WINDOW_AUTOSIZE);
    imshow(TurnWind, TurnSections);


    std::cout << "Press any Key to continue" << std::endl;
    cv::waitKey(0);

    cv::destroyWindow(StraightWind);
    cv::destroyWindow(TightTurntWind);
    cv::destroyWindow(TurnWind);



    //fill the SectionVecX - SectionVecY
    CompleteVect(StraightSections, StraightRect, StraightVecX, StraightVecY);

    CompleteVect(TurnSections, TurnRect, TurnVecX, TurnVecY);

    CompleteVect(TightTurnSections, TightTurnRect, TightTurnVecX, TightTurnVecY);
};


void Pilot::getPath(std::vector<cv::Point> points, cv::Mat& drawing_path)
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

        //std::cout << "Points " << i << " -- ( " << ordered_point_path[i].x << " ; " << ordered_point_path[i].y << " ) ";
        //std::cout << " -- FILTERED = ( " << filtered_ordered_point_path[i].y << " ; " << filtered_ordered_point_path[i].y << " )" << std::endl;

        /*cv::circle(drawing_path, ordered_point_path[prev], 1, cv::Scalar(125), 1, cv::LINE_8);
        cv::circle(drawing_path, ordered_point_path[i], 1, cv::Scalar(125), 1, cv::LINE_8);

        line(drawing_path, ordered_point_path[prev], ordered_point_path[i], cv::Scalar(125), 1, cv::LINE_8);*/


        cv::circle(drawing_path, filtered_ordered_point_path[prev], 1, cv::Scalar(255), 1, cv::LINE_8);
        cv::circle(drawing_path, filtered_ordered_point_path[i], 1, cv::Scalar(255), 1, cv::LINE_8);

        line(drawing_path, filtered_ordered_point_path[prev], filtered_ordered_point_path[i], cv::Scalar(255), 1, cv::LINE_8);
        cv::imshow("Path", drawing_path);
        cv::waitKey(0);

    }

    
    //cv::waitKey(0);
    cv::destroyWindow("Path");




    //check for the correct rotation


    cv::Point p;
    p.x = Startgrid.x + Startgrid.width / 2;
    p.y = Startgrid.y + Startgrid.height / 2;

    double shortestDist = 999999;
    shortestDistIdx = -1;

    for (int i = 0; i < filtered_ordered_point_path.size(); i++)
    {
        
        double dist = sqrt(pow(p.x - filtered_ordered_point_path[i].x, 2) + pow(p.y - filtered_ordered_point_path[i].y, 2));

        if (dist < shortestDist)
        {
            shortestDistIdx = i;
            shortestDist = dist;
        }
    }
    std::cout<<std::endl;

    int val = shortestDistIdx + 3;
    if (val >= filtered_ordered_point_path.size())
        val -= filtered_ordered_point_path.size();


    if (filtered_ordered_point_path[shortestDistIdx].x < filtered_ordered_point_path[val].x)
    {
        std::cout << "Error encoding points -- reversing order" << std::endl;

        for (int i = 0; i < filtered_ordered_point_path.size() / 2; i++)
        {
            cv::Point p = filtered_ordered_point_path[i];
            filtered_ordered_point_path[i] = filtered_ordered_point_path[filtered_ordered_point_path.size() - 1 - i];
            filtered_ordered_point_path[filtered_ordered_point_path.size() - 1 - i] = p;
        }
    }
    else
        std::cout << "Path OK -- correcte order" << std::endl;
}


void Pilot::SavePath()
{
    //used to get the mouvement of the car
    std::vector<std::vector<cv::Point>> mvt;

    //used to locate the car at specific point
    std::vector<cv::Point> point_path;

    std::string wind = "Video";
    cv::namedWindow(wind, cv::WINDOW_AUTOSIZE);

    std::cout << "Once you have made 2 lap of the track, press Space to continue processing" << std::endl;

    cv::Mat image, stream;
    bool state = p_cap.read(image);
    Detection::GetView(image, stream, p_M, p_cadre, p_mask);
    image.release();

    double LearningRate = 0.01;
    mvt = Detection::BackgroundSubstraction(stream, LearningRate);
    bool first = true;

    

    while (state)
    {
        state = p_cap.read(image);
        Detection::GetView(image, stream, p_M, p_cadre, p_mask);
        image.release();

        mvt = Detection::BackgroundSubstraction(stream, LearningRate);

        if (mvt.size() != 0)
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


bool Pilot::CheckPath(cv::Point p, std::vector<int>SectionVecX, std::vector<int>SectionVecY)
{
    bool section = false;
    std::vector<int> SectionVecI;

    for (int i = 0; i < SectionVecX.size() - 1; i += 2)
    {
        if (SectionVecX[i] < p.x && p.x < SectionVecX[i + 1])
        {
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
                section = true;
            }
        }
    }

    SectionVecI.clear();
    return section;
};


//from a point, say
int Pilot::getSection(cv::Point2f p)
{
    cv::Mat Black = cv::Mat::zeros(CarPath.size(), CV_8U);
    cv::circle(Black, p, 1, cv::Scalar(255));
    cv::Mat result;


    cv::bitwise_and(Black, Black, result, StraightSections);
    if (cv::countNonZero(result) == 0)
    {
        cv::bitwise_and(Black, Black, result, TurnSections);
        if (cv::countNonZero(result) == 0)
        {
            cv::bitwise_and(Black, Black, result, TightTurnSections);
            if (cv::countNonZero(result) == 0)
                return INVALID;
            else
                return TIGHTURN;
        }
        else
            return TURN;

    }
    else
        return STRAIGHT;
};


void Pilot::train()
{
    //fil the vectors PointsSection

    std::pair<cv::Point2f, int> PointSection;

    std::vector<std::pair<int, double>> temp;

    //std::cout << "filtered_ordered_point_path.size() = " << filtered_ordered_point_path.size() << std::endl;

    for (int i = 0; i < filtered_ordered_point_path.size(); i++)
    {
        PointSection.first = filtered_ordered_point_path[i];
        PointSection.second = getSection(filtered_ordered_point_path[i]);

        PointsSection.push_back(PointSection);

    }
};


void Pilot::drive()
{
    cv::Mat image, stream;
    

    bool state = p_cap.read(image);
    Detection::GetView(image, stream, p_M, p_cadre, p_mask);
    image.release();

    //calibration of camera
    int mid_height = stream.size().height / 2;
    int mid_width = stream.size().width / 2;

    cv::Point p0;
    p0.x = mid_width - (30 * 2 + 1);
    p0.y = mid_height - (20 * 2 + 1);

    cv::Point p1;
    p1.x = mid_width + (30 * 2 + 1);
    p1.y = mid_height - (20 * 2 + 1);

    cv::Point p2;
    p2.x = mid_width + (30 * 2 + 1);
    p2.y = mid_height + (20 * 2 + 1);

    p_pointVec.push_back(p0);
    p_pointVec.push_back(p1);
    p_pointVec.push_back(p2);

    double distx =sqrt(pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2));
    double disty = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));

    factorPxl = ((20/disty)+(30/distx))/2;

    std::cout << "30 cm --> " << distx << " pxl" << std::endl;
    std::cout << "20 cm --> " << disty << " pxl" << std::endl;
    std::cout << factorPxl <<" cm --> 1 pxl" << std::endl;




    cv::Mat trackMask;
    cv::resize(p_mask, trackMask, cv::Size(), 2.0, 2.0);


    //car 1 -- IA
    cv::Mat trackCar1;
    cv::bitwise_and(stream, stream, trackCar1, CarPath);

    double LearningRate = 0.01;
    std::vector<std::vector<cv::Point>> car1;

    std::string wind_Vid = "Stream";
    cv::namedWindow(wind_Vid, cv::WINDOW_AUTOSIZE);

    std::cout << "==== START ====" << std::endl;

    int valToSend = 50;
    int prvdata = 0x00;    

    int PosT = -1;
    int shift_pxl = Shift_cm * (1/factorPxl);
    int shift;



    //compute first PosT index
    cv::Point firstP;

    firstP.x = Startgrid.x + Startgrid.width / 2;
    firstP.y = Startgrid.y + Startgrid.height / 2;


    double shortestDist = 999999;
    int shortestDistIdx = -1;

    for (int i = 0; i < PointsSection.size(); i++)
    {

        //std::cout << "val = " << val << std::endl;
        double dist = sqrt(pow(firstP.x - PointsSection[i].first.x, 2) + pow(firstP.y - PointsSection[i].first.y, 2));

        if (dist < shortestDist)
        {
            shortestDistIdx = i;
            shortestDist = dist;
        }
    }

    PosT = shortestDistIdx;

    for (int i = 0; i < PointsSection.size(); i++)
    {
        int val= shortestDistIdx + i;

        if (val >= PointsSection.size())
            val -= PointsSection.size();

        double dist = sqrt(pow(PointsSection[shortestDistIdx].first.x - PointsSection[val].first.x, 2) + pow(PointsSection[shortestDistIdx].first.y - PointsSection[val].first.y, 2));

        if (dist >= shift_pxl)
        {
            if (val > shortestDistIdx)
                shift = val - shortestDistIdx;
            else
                shift = val + (PointsSection.size() - shortestDistIdx);

            break;
        }
            

    }

    //start time of computing to schow number of fps 
    std::string fps;
    int frame_counter = 0;
    double startTime = (double)cv::getTickCount();




    //test
    for (int i = 0; i < p_pointVec.size(); i++)
    {
        std::cout << "Point " << i << " :( " << p_pointVec[i].x << "; " << p_pointVec[i].y << " );" << std::endl;
    }
    std::cout << "dim of image stream: " << stream.cols << " x " << stream.rows << std::endl;

    std::cout << "Startgrid position to (" << Startgrid.x << ";" << Startgrid.y << ") with dim "<< Startgrid.width<<" x "<< Startgrid.height<<std::endl;



    //

    cv::Point Prevp = firstP;
    while (state)
    {
        double start = (double)cv::getTickCount();
        state = p_cap.read(image);
        Detection::GetView(image, stream, p_M, p_cadre, p_mask);
        image.release();

        cv::bitwise_and(stream, stream, trackCar1, CarPath);
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

        

        cv::Mat point = cv::Mat::zeros(stream.size(), CV_8U);

        if (p != cv::Point(-1, -1) && PosT != -1)
        {
            /*straight = CheckPath(p, StraightVecX, StraightVecY);
            turn = CheckPath(p, TurnVecX, TurnVecY);
            tightturn = CheckPath(p, TightTurnVecX, TightTurnVecY);*/


            double shortestDist = 999999;
            int shortestDistIdx = -1;

            for (int i = PosT; i <= PosT + shift; i++)
            {
                int val;

                if (i >= PointsSection.size())
                    val = i - PointsSection.size();
                else
                    val = i;

                cv::circle(point, PointsSection[val].first, 2, cv::Scalar(125), 2);
                cv::circle(point, Prevp, 2, cv::Scalar(255), 2);
            }


            for (int i = PosT; /*i <= PosT + shift*/; i++)
            {
                int val;
                
                if (i >= PointsSection.size())
                    val = i - PointsSection.size();
                else
                    val = i;

                //std::cout << "val = " << val << std::endl;
                double dist = sqrt(pow(p.x - PointsSection[val].first.x,2)+pow(p.y - PointsSection[val].first.y, 2));

                if (dist < shortestDist)
                {
                    shortestDistIdx = val;
                    shortestDist = dist;
                }
                else if (shortestDistIdx != -1)
                {
                    dist = sqrt(pow(PointsSection[shortestDistIdx].first.x - PointsSection[val].first.x, 2) + pow(PointsSection[shortestDistIdx].first.y - PointsSection[val].first.y, 2));

                    if (dist >= shift_pxl)
                    {
                        if (val > shortestDistIdx)
                            shift = val - shortestDistIdx;
                        else
                            shift = val + (PointsSection.size() - shortestDistIdx);
                        break;
                    }
                }

                
            }

            if (shortestDistIdx != -1)
            {
                PosT = shortestDistIdx;
            }
            else
            {
                std::cout << "Car point nor found" << std::endl;
            }


            int nextPos;

            if (PosT + shift >= PointsSection.size())
                nextPos = PosT + shift - PointsSection.size();
            else
                nextPos = PosT + shift;


            if (PointsSection[nextPos].second == STRAIGHT)
                straight = true;
            else if (PointsSection[nextPos].second == TURN)
                turn = true;
            else if (PointsSection[nextPos].second == TIGHTURN)
                tightturn = true;

           
            if (straight)
            {
                valToSend = 100;
            }

            if (turn)
            {
                valToSend = 90;
            }     

            if (tightturn)
            {
                valToSend = 70;
            }

            if (valToSend != prvdata)
            {
                char data = valToSend;
                bridge->writeSerialPort(&data, 1);
                prvdata = data;
            }

        }

        Prevp = p;

        for (int i = 0; i < p_pointVec.size(); i++)
        {
            cv::circle(stream, p_pointVec[i], 2, cv::Scalar(0,255,0), 2);
        }

        //draw all rectangles of the sections on the screen
        for (int i = 0; i < StraightRect.size(); i++)
        {
            cv::rectangle(stream, StraightRect[i], cv::Scalar(0, 255, 255), 2);
        }
        for (int i = 0; i < TurnRect.size(); i++)
        {
            cv::rectangle(stream, TurnRect[i], cv::Scalar(0, 255, 0), 2);
        }
        for (int i = 0; i < TightTurnRect.size(); i++)
        {
            cv::rectangle(stream, TightTurnRect[i], cv::Scalar(0, 0, 255), 2);
        }
        cv::rectangle(stream, Startgrid, cv::Scalar(255, 255, 255), 2);


        //shoow fps
        frame_counter++;
        if (((double)cv::getTickCount() - startTime) / cv::getTickFrequency() >= 1.0)
        {
            fps = std::to_string(frame_counter) + " FPS";

            frame_counter = 0;
            startTime = cv::getTickCount();
        }
        cv::putText(stream, fps, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        //show time to process
        double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
        std::string txt = std::to_string(time*1000) + " ms";
        cv::putText(stream, txt, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);



        cv::imshow(wind_Vid, stream);
        cv::imshow("car1 IA", trackCar1);
        cv::imshow("Points", point);

        stream.release();
        trackCar1.release();
        point.release();


        if (cv::waitKey(10) == 27)
            state = false;
    }


    char data = 0x00;
    bridge->writeSerialPort(&data, 1);
    
    trackMask.release();
    
};