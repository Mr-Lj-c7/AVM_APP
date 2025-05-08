#ifndef AVM_H_
#define AVM_H_

#include "FisheyeCameraModel.h"
#include "CalibrateCamera.h"
#include "BirdView.h"
#include "UtilsView.h"
#include "ThreadQueue.h"
#include <yaml-cpp/yaml.h> 
#include <chrono>
#include <thread>
#include <functional>

// #define CALIBRATE_CAMERA
#define WHITE_BALANCE 0
#define DEBUG 1
extern std::atomic<bool> global_stop;  // 外部变量

struct DEVICE
{
    std::string front_ = "front";
    std::string back_ = "back";
    std::string left_ = "left";
    std::string right_ = "right";
};

class Avm
{
    private:
        DEVICE camera_;

        std::string car_model_path;
        std::string result_out_path;
        std::string front_video_path;
        std::string back_video_path;
        std::string left_video_path;
        std::string right_video_path;
        std::string stitched_out_path;
        std::string white_balance_stitched_out_path;

        cv::Mat frame_front, frame_back, frame_left, frame_right;
        cv::Mat frame_stitched, frame_whited;

        long cnt = 0;

    public:
        Avm();
        ~Avm();
        bool loadParams();
        void cameraCalibrate(const std::string &camera_name, 
            const cv::Size &board_size, 
            const cv::Size &square_size);

        void cameraProcess(cv::VideoCapture& video,
            FisheyeCameraModel& camera,
            ThreadSafeQueue<cv::Mat>& output_queue,
            std::atomic<bool>& stop_flag);
        void run();
};

#endif
