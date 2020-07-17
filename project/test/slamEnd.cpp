#include "slamBase.h"
#include "realsense.h"
#include <librealsense2/rs.hpp>

// #include "g2o/types/slam3d/types_slam3d.h"
// #include "g2o/core/sparse_optimizer.h"
// #include "g2o/core/block_solver.h"
// #include "g2o/core/factory.h"
// #include "g2o/core/optimization_algorithm_factory.h"
// #include "g2o/core/optimization_algorithm_gauss_newton.h"
// #include "g2o/core/robust_kernel.h"
// #include "g2o/core/robust_kernel_factory.h"
// #include "g2o/solvers/eigen/linear_solver_eigen.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

FRAME readFrame( RealSense& realsense, ParameterReader& pd );
double normofTransform( cv::Mat rvec, cv::Mat tvec );

cv::Affine3d cameraPose;
Eigen::Isometry3d cameraPoseT;
int main(void)
{
    RealSense realsense;
    ParameterReader pd;
    
    cout<<"Initializing ..."<<endl;
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
    int lastIndex = 0;

    typedef g2o::BlockSolver_6_3 SlamBlockSolver; // BA(Bundle Adjustment 3D SLAM)
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    // std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    // std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));


    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm( solver );
    globalOptimizer.setVerbose( false );

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() );
    v->setFixed( true ); 
    globalOptimizer.addVertex( v );

    while (1)
    {
        currIndex++;

        int check = cv::waitKey(1);
        if (check == 's')
        {
        }
        else if (check == 'q')
            break;


        cout<<endl<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame(realsense, pd);
        computeKeyPointsAndDesp(currFrame);
        cv::Mat imgShow1;
        cv::drawKeypoints(currFrame.rgb, currFrame.kp, imgShow1, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cv::imshow("currFrame", imgShow1);
        cv::Mat imgShow2;
        cv::drawKeypoints(lastFrame.rgb, lastFrame.kp, imgShow2, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cv::imshow("lastFrame", imgShow2);
        
        if(lastFrame.kp.size()<=0 || currFrame.kp.size()<=0){
            cout << " kp error" << endl;
            continue;
        }
        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
        if(result.inliers<min_inliers){
            cout << " inlier error" << endl;
            continue;
        }
        if(result.goodMatches.size() <= min_good_match){
            cout << "goodMatches size   error" << endl;
            continue;
        }
        
        double norm = normofTransform(result.rvec, result.tvec);
        if(norm>=max_norm){
            cout << "norm   error" << endl;
            continue;
        }
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        cv::Affine3d M = affine2Mat(result.rvec, result.tvec);

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex ); // 정점 ID 설정
        // v->setEstimate( Eigen::Isometry3d::Identity() );
        v->setEstimate( cameraPoseT );
        globalOptimizer.addVertex(v); // 정점 추가
        g2o::EdgeSE3* edge = new g2o::EdgeSE3(); // Edge의 각각 꼭짓점
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );

        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        edge->setInformation( information );
        edge->setMeasurement( T );
        globalOptimizer.addEdge(edge);
        if(currIndex == 1){
            cameraPose = M;
            cameraPoseT = T;
        }else{
            // cameraPose  = M.concatenate(cameraPose);
            cameraPoseT = cameraPoseT*T;
            cameraPose = affine2Eigen(cameraPoseT);
        }
        cout<<"(eigen)T="<<cameraPoseT.matrix()<<endl;
        
        cv::imshow("image", currFrame.rgb);
        vis.setWidgetPose( "Camera", cameraPose);
        vis.spinOnce(1, false);

        lastFrame = currFrame;
        lastIndex = currIndex;
    }

    // pcl::io::savePCDFile("data/result.pcd", *cloud);

    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 );
    globalOptimizer.save( "./result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();
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