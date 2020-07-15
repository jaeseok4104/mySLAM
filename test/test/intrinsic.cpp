#include <librealsense2/rs.hpp>
#include <iostream>
using namespace std;

int main()
{
    //get camera parameters
    rs2::pipeline pipe;
    rs2::pipeline_profile selection = pipe.start();
    // auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
    //                         .as<rs2::video_stream_profile>();
    auto color_stream = selection.get_stream(RS2_STREAM_COLOR)
                            .as<rs2::video_stream_profile>();
    auto resolution = std::make_pair(color_stream.width(), color_stream.height());
    // auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());

    auto i = color_stream.get_intrinsics();
    // auto i = depth_stream.get_extrinsics_to(color_stream);

    auto principal_point = std::make_pair(i.ppx, i.ppy);
    auto focal_length = std::make_pair(i.fx, i.fy);
    // rs2_distortion model = i.model;
    //print the parameters
    // cout << "rotation" << endl;
    // for (int a = 0; a < 9; a++)
    // {
    //     cout << i.rotation[a]<< "\t";
    //     if(((a+1)%3) == 0) cout << "\n";
    // }
    // cout << "translation" << endl;
    // for (int a = 0; a <3; a++)
    // {
    //     cout << i.translation[a] << endl;
    // }
    cout << "Resolution: " << resolution.first << "," << resolution.second << endl;
    cout << "Principle point: " << principal_point.first << "," << principal_point.second << endl;
    cout << "Focal length: " << focal_length.first << "," << focal_length.second << endl;
    return 0;
}