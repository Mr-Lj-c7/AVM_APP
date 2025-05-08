#include "Avm.h"

int main(int argc, char** argv)
{
    Avm avm;
    #ifdef CALIBRATE_CAMERA
        avm.cameraCalibrate(argv[1], cv::Size(9, 6), cv::Size(10, 10));
    #endif
    try{
        avm.run();
    } catch(std::exception& e){
        std::cerr << "AVM failed." << e.what() << std::endl;
    }
    return 0;
}