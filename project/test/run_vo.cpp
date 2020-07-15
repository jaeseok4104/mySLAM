#include "slamBase.h"
#include "realsense.h"
#include <librealsense2/rs.hpp>

///////Eigen/////// SO --> SE
#include <Eigen/Core>
#include <Eigen/Geometry>

FRAME readFrame( RealSense& realsense, ParameterReader& pd );
double normofTransform( cv::Mat rvec, cv::Mat tvec );

cv::Affine3d cameraPose;
cv::Matx33d cameraPoseR;
cv::Vec3d cameraPoseT;
int main(void)
{
    RealSense realsense;
    ParameterReader pd;
    
    cout<<"test"<<endl;
    FRAME lastFrame = readFrame(realsense, pd);

    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    // pcl::visualization::CloudViewer viewer("viewer");

    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );


    //visualize check
    // bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi(pd.getData("min_inliers").c_str());
    double max_norm = atof(pd.getData("max_norm").c_str());
    double min_good_match = atof(pd.getData("min_good_match").c_str());

    int currIndex = 0;
    while (1)
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame(realsense, pd);
        computeKeyPointsAndDesp(currFrame);
        cout<<"  test   point"<<endl;
        if(lastFrame.kp.size()<=0 || currFrame.kp.size()<=0){
            lastFrame = currFrame;
            continue;
        }
        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
        if(result.inliers<min_inliers){
            // lastFrame = currFrame;
            continue;
        }
        cout<<"  test   point"<<endl;
        if(result.goodMatches.size() <= min_good_match){
            // lastFrame = currFrame;
            continue;
        }
        
        double norm = normofTransform(result.rvec, result.tvec);
        if(norm>=max_norm){
            // lastFrame = currFrame;
            continue;
        }
        // Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        // cv::Affine3d M = affine2Mat(currentVec.rvec, currentVec.tvec);
        cv::Affine3d M = affine2Mat(result.rvec, result.tvec);
        if(currIndex == 0){
            cameraPose = M;
        }

        cout<<"T="<<M.matrix<<endl;
        cout<<"T_c="<<cameraPose.matrix<<endl;
        
        cameraPose=cameraPose.concatenate(M);
        cv::imshow("image", currFrame.rgb);
            int check = cv::waitKey(1);
            if (check == 's')
            {
            }
            else if (check == 'q')
                break;  
        vis.setWidgetPose( "Camera", cameraPose);
        vis.spinOnce(1, false);

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