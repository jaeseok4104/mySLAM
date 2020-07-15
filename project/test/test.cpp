#include <iostream>
#include <stdlib.h>
#include "slamBase.h"
using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>
//librealsense2
#include <librealsense2/rs.hpp>

int main(int argc, char **argv)
{
    //realsense setting
    rs2::pipeline pipe;
    rs2::pipeline_profile pipeline_profile;

    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    pipeline_profile = pipe.start(cfg);

    ////img buffer/////
    cv::Mat rgb1, rgb2;
    cv::Mat depth1, depth2;

    //Camera_Intrinsic_Parameters
    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = 327.4;
    C.cy = 239.9;
    C.fx = 613.5;
    C.fy = 613.6;
    C.scale = 1000.0;
    /////////////////////////////////////
    
    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}};

    while (1)
    {
        rs2::frameset frames = pipe.wait_for_frames();

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::video_frame depth_frame = frames.get_depth_frame();

        rgb1 = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        if(rgb2.size().height == 0 && rgb2.size().width == 0){
            rgb2 = rgb1; // first frame fix
            continue;
        }
        
        cv::Mat exdepth = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        exdepth.convertTo(depth1, CV_8UC1, 255.0/65536.0);
        // if(depth2.size().height == 0 && depth2.size().width == 0){
        //     depth2 = depth1; // first frame fix
        //     continue;
        // }
        
        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> descriptor;
        
        detector = cv::ORB::create();
        descriptor = cv::ORB::create();

        vector<cv::KeyPoint> kp1, kp2;
        detector->detect(rgb1, kp1);
        detector->detect(rgb2, kp2);
        cout << "Key points of two images: " << kp1.size() << ", " << kp2.size() << endl;

        cv::Mat imgShow;
        cv::drawKeypoints(rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow("keypoints", imgShow);
        cv::imshow("normal", rgb1);
        cv::imshow("depth", depth1);
        // cv::imwrite("./data/keypoints.png", imgShow);

        /////////////Matching correspondences//////////
        cv::Mat desp1, desp2;
        descriptor->compute(rgb1, kp1, desp1);
        descriptor->compute(rgb2, kp2, desp2);

        vector<cv::DMatch> matches;
        cv::BFMatcher matcher;
        if(kp1.size()>0 && kp2.size()>0)
            matcher.match(desp1, desp2, matches);
        cout << "Find total " << matches.size() << " matches." << endl;

        cv::Mat imgMatches;
        cv::drawMatches(rgb1, kp1, rgb2, kp2, matches, imgMatches);
        cv::imshow("matches", imgMatches);
        // cv::imwrite("./data/matches.png", imgMatches);

        vector<cv::DMatch> goodMatches;
        double minDis = 9999; // 임의의 거리
        double sumDis = 0;
        for (size_t i = 0; i < matches.size(); i++){
            sumDis += matches[i].distance;
        }
        minDis = sumDis / (matches.size());
        cout << "minDis:" << minDis << endl;
        for (size_t i = 0; i < matches.size(); i++){
            if (matches[i].distance < minDis)
                goodMatches.push_back(matches[i]);
        }

        cout << "good matches: " << goodMatches.size() << endl;

        vector<cv::Point3f> pts_obj;
        vector<cv::Point2f> pts_img;

        for (size_t i = 0; i < goodMatches.size(); i++)
        {
            cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
            ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
            if (d == 0)
                continue;
            pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));

            // 将(u,v,d)转成(x,y,z)
            cv::Point3f pt(p.x, p.y, d);
            cv::Point3f pd = point2dTo3d(pt, C);
            pts_obj.push_back(pd);
        }

        double camera_matrix_data[3][3] = {
            {C.fx, 0, C.cx},
            {0, C.fy, C.cy},
            {0, 0, 1}};

        cout << "solving pnp" << endl;
        cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
        cv::Mat rvec, tvec, inliers;
        cv::Mat R_pos;

        int check = cv::waitKey(1);
        if (check == 's')
        {
        }
        else if (check == 'q')
            break;
        rgb2 = rgb1;
        // depth2 = depth1;
        system("clear");
    }


    return 0;
}
