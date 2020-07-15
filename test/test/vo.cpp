#include "slamBase.h"
#include "realsense.h"
#include <librealsense2/rs.hpp>

///////Eigen/////// SO --> SE
#include <Eigen/Core>
#include <Eigen/Geometry>

FRAME readFrame( RealSense& realsense, ParameterReader& pd );
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main(void)
{
    RealSense realsense;
    ParameterReader pd;
    
    cout<<"test"<<endl;
    FRAME lastFrame = readFrame(realsense, pd);

    FRAME frame1, frame2;
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");


    //visualize check
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi(pd.getData("min_inliers").c_str());
    double max_norm = atof(pd.getData("max_norm").c_str());
    double min_good_match = atof(pd.getData("min_good_match").c_str());

    int currIndex = 0;
    while (1)
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame(realsense, pd);
        computeKeyPointsAndDesp(currFrame);
        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
        if(result.inliers<min_inliers)
            continue;
        cout<<"  test   point    "<<endl;
        if(result.goodMatches.size() < min_good_match)
            continue;
        
        double norm = normofTransform(result.rvec, result.tvec);
        if(norm>=max_norm)
            continue;
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cout<<"T="<<T.matrix()<<endl;

        cloud = joinPointCloud(cloud, currFrame, T, camera);

        if(visualize == true)
            viewer.showCloud(cloud);
        lastFrame = currFrame;
        currIndex++;
    }

    // pcl::io::savePCDFile("data/result.pcd", *cloud);
}

FRAME readFrame( RealSense& realsense, ParameterReader& pd )
{
    FRAME f;

    realsense.updateFrame();
    realsense.updateColor();
    realsense.updateDepth();
    realsense.drawColor();
    realsense.drawDepth();

    f.rgb = realsense.color_mat;
    cv::Mat depth = realsense.depth_mat;
    depth.convertTo(f.depth, CV_8UC1, 255.0/65536.0);

    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}