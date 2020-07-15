// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
    
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    
    Mat color_equal;
    
    while(1)
    {
            frames = pipe.wait_for_frames();
        //Get each video_frame
        rs2::video_frame color_frame = frames.get_depth_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(1280, 720), CV_16UC1, (void *)color_frame.get_data(), Mat::AUTO_STEP);
       
        color.convertTo(color_equal, CV_8UC1, 1000);
        // Display in a GUI
        equalizeHist(color_equal, color_equal);
        // applyColorMap(color, color, COLORMAP_JET);
        imshow("Display Image", color);
        imshow("Display Image_equalize", color_equal);

        char check = waitKey(1);
        if(check == 's'){

        }
        else if(check == 'q')
            break;
    }

    return 0;
}