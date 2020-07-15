#include <iostream>
#include <sstream>
#include "realsense.h"

int main( int argc, char* argv[] )
{
    try{
        RealSense realsense;
        realsense.run();
    } catch( std::exception& ex ){ // 예외처리
        std::cout << ex.what() << std::endl;
    }

    return 0;
}