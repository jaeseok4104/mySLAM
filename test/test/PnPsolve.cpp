#include "slamBase.h"
#include "realsense.h"
#include <librealsense2/rs.hpp>

///////Eigen/////// SO --> SE
#include <Eigen/Core>
#include <Eigen/Geometry>


int main(void)
{
    rs2::pipeline pipe;
    rs2::pipeline_profile pipeline_profile;

    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);


    pipeline_profile = pipe.start(cfg);

    ParameterReader pd;
    FRAME frame1, frame2;

    CAMERA_INTRINSIC_PARAMETERS camera;
        camera.fx = atof(pd.getData("camera.fx").c_str());
        cout << camera.fx<<endl;
        camera.fy = atof(pd.getData("camera.fy").c_str());
        cout << camera.fy<<endl;
        camera.cx = atof(pd.getData("camera.cx").c_str());
        cout << camera.cx<<endl;
        camera.cy = atof(pd.getData("camera.cy").c_str());
        cout << camera.cy<<endl;
        camera.scale = atof(pd.getData("camera.scale").c_str());

        rs2::frameset frames = pipe.wait_for_frames();

        rs2::video_frame color_frame = frames.get_color_frame();

        frame2.rgb = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        PointCloud::Ptr output(new PointCloud());
        while (1)
        {
            rs2::frameset frames = pipe.wait_for_frames();

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::video_frame depth_frame = frames.get_depth_frame();
            frame1.rgb = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
            if (frame2.rgb.size().height == 0 && frame2.rgb.size().width == 0)
                frame2.rgb = frame1.rgb;

            frame1.depth = cv::Mat(cv::Size(640, 480), CV_16SC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            computeKeyPointsAndDesp(frame1);
            computeKeyPointsAndDesp(frame2);

            cout << "solving pnp" << endl;

            RESULT_OF_PNP result = estimateMotion(frame1, frame2, camera);
            cout << "test point" << endl;
            cout << result.rvec << endl
                 << result.tvec << endl;

            //Rodrigues를 SO로 SO(3)를 SE(3)로 translation
            cv::Mat R;
            cv::Rodrigues(result.rvec, R);
            Eigen::Matrix3d r;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r(i, j) = R.at<double>(i, j);

            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

            Eigen::AngleAxisd angle(r);
            T = angle;
            T(0, 3) = result.tvec.at<double>(0, 0);
            T(1, 3) = result.tvec.at<double>(1, 0);
            T(2, 3) = result.tvec.at<double>(2, 0);

            PointCloud::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
            PointCloud::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);

            cout << "combining clouds" << endl;
            pcl::transformPointCloud(*cloud1, *output, T.matrix());
            *output += *cloud2;

            cout << "Final result saved." << endl;

            pcl::visualization::CloudViewer viewer("viewer");
            viewer.showCloud(output);
            frame2.rgb = frame1.rgb;
    }
    pcl::io::savePCDFile("data/result.pcd", *output);
}