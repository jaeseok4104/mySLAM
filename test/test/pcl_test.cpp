#include <iostream>
#include <string>
#include <ctime>

// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/viz3d.hpp>

//PCL header file
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Core>

//namespace
using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

//intrinsic parameter
const double camera_factor = 1000;
const double camera_cx = 651.118774414062;
const double camera_cy = 359.867980957031;
const double camera_fx = 920.3212890625;
const double camera_fy = 920.426818847656;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    rs2::pipeline_profile pipeline_profile;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipeline_profile = pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    //realsense img to Mat
    cv::Mat rgb, depth;

    //pcl rgd and Depth
    frames = pipe.wait_for_frames();
    //Get each video_frame

    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::video_frame depth_frame = frames.get_depth_frame();

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
    Mat rs_depth(Size(640, 480), CV_16SC1, (void *)depth_frame.get_data(), Mat::AUTO_STEP);
    rs_depth.convertTo(rs_depth, CV_8U, 255.0 / 10000.0, 0);

    // Display in a GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("color Image", color);
    imshow("rs_depth Image", rs_depth);

    // char check = waitKey(1);
    // if(check == 's'){}
    // else if(check == 'q')
    //     break;
    //////////////////////////////////////////////////////////////////////////////////////////
    rgb = color;
    depth = rs_depth;

    PointCloud::Ptr cloud(new PointCloud);

    for (int m = 0; m < rs_depth.rows; m++)
        for (int n = 0; n < rs_depth.cols; n++)
        {
            ushort d = rs_depth.ptr<ushort>(m)[n];

            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            cloud->points.push_back(p);
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
    // 清除数据并退出
    cloud->points.clear();
    cout << "Point cloud saved." << endl;

    waitKey(0);
    return 0;
}