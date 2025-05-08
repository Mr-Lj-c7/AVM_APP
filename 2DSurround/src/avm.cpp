#include "FisheyeCameraModel.h"
#include "CalibrateCamera.h"
#include "BirdView.h"
#include "UtilsView.h"
#include "ThreadQueue.h"
#include "Avm.h"

std::atomic<bool> global_stop(false);

Avm::Avm()
{
    const auto config_ = loadParams();
    if (!config_) {
        std::cerr << "load params failed" << std::endl;
        return;
    }
}

Avm::~Avm(){}

void Avm::cameraCalibrate(const std::string &camera_name, const cv::Size &board_size, const cv::Size &square_size)
{
    CalibrateCamera camera0(camera_name, board_size, square_size);
        
    camera0.extract_corners();
    camera0.calibrate_process();
    camera0.calc_project_matrix();
    camera0.save_calibrated_yaml();
}

void Avm::cameraProcess(cv::VideoCapture& video,
    FisheyeCameraModel& camera,
    ThreadSafeQueue<cv::Mat>& output_queue,
    std::atomic<bool>& stop_flag) 
{
    try{
        cv::Mat frame, undistorted, perspective;
        while (!stop_flag.load()) {
            if (!video.read(frame)) {
                stop_flag = true;
                break;
            }
            // 处理帧：去畸变 + 透视变换
            camera.undistort_remap(frame, undistorted);
            camera.project_warp_perspective(undistorted, perspective);
            // 将处理后的帧加入队列
            if(stop_flag.load()) break;
            output_queue.push(perspective.clone());
        }
    }
    catch(...){
        std::cerr << "Unknown exception in camera thread" << std::endl;
        stop_flag = true;
    }  
}

bool Avm::loadParams()
{
    try {
        YAML::Node config = YAML::LoadFile("../config/fisheye/config.yaml");
        // YAML::Node config = YAML::LoadFile("./config/fisheye/config.yaml");  // docker 

        car_model_path = config["car_model_path"].as<std::string>();
        result_out_path = config["result_out_path"].as<std::string>();
        front_video_path = config["front_video_path"].as<std::string>();
        back_video_path = config["back_video_path"].as<std::string>();
        left_video_path = config["left_video_path"].as<std::string>();
        right_video_path = config["right_video_path"].as<std::string>();
        stitched_out_path = config["stitched_out_path"].as<std::string>();
        white_balance_stitched_out_path = config["white_balance_stitched_out_path"].as<std::string>();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return false;
    }
}

void Avm::run()
{
    std::cerr << "avm app run" << std::endl;
    // camera front back left right
    FisheyeCameraModel camera_front(camera_.front_);
    FisheyeCameraModel camera_back(camera_.back_);
    FisheyeCameraModel camera_left(camera_.left_);
    FisheyeCameraModel camera_right(camera_.right_);
    // load camera params
    camera_front.load_camera_params();
    camera_back.load_camera_params();
    camera_left.load_camera_params();
    camera_right.load_camera_params();
    // update undistort maps
    camera_front.update_undistort_maps();
    camera_back.update_undistort_maps();
    camera_left.update_undistort_maps();
    camera_right.update_undistort_maps();
    // read camera
    cv::VideoCapture video_front(front_video_path);
    cv::VideoCapture video_back(back_video_path);
    cv::VideoCapture video_left(left_video_path);
    cv::VideoCapture video_right(right_video_path);
    if (!video_front.isOpened() || !video_back.isOpened() ||
        !video_left.isOpened() || !video_right.isOpened()){
            std::cerr << "video is open failed!" << std::endl;
            return;
        }
    if (DEBUG){
        std::cout << "camera: " << camera_.front_ << ", frame: " << video_front.get(cv::CAP_PROP_FRAME_COUNT) 
            << ", " << "W x H: " << video_front.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
            << video_front.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        std::cout << "camera: " << camera_.back_  << ", frame: " << video_back.get(cv::CAP_PROP_FRAME_COUNT) 
                << ", " << "W x H: " << video_back.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
                << video_back.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        std::cout << "camera: " << camera_.left_  << ", frame: " << video_left.get(cv::CAP_PROP_FRAME_COUNT) 
                << ", " << "W x H: " << video_left.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
                << video_left.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        std::cout << "camera: " << camera_.right_ << ", frame: " << video_right.get(cv::CAP_PROP_FRAME_COUNT) 
                << ", " << "W x H: " << video_right.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
                << video_right.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    }
    // car model 
    cv::Mat car_model;
    car_model = cv::imread(car_model_path);
    BirdView bv(car_model);
    UtilsView uv;
    // stitch 4 frames
    ParamSettings params;
    cv::VideoWriter stitched_output(result_out_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(params.total_w, params.total_h));
    // 4 camera threads
    const std::chrono::seconds wait_timeout_duration(5);  // 5秒等待超时
    ThreadSafeQueue<cv::Mat> queue_front(global_stop), queue_back(global_stop), queue_left(global_stop), queue_right(global_stop);
    if (video_front.isOpened() && video_back.isOpened() && 
        video_left.isOpened() && video_right.isOpened())
    {
        // 启动四个摄像头处理线程
        std::thread camera_threads[4];  // front, back, left, right
        camera_threads[0] = std::thread(std::bind(&Avm::cameraProcess, this, std::ref(video_front), std::ref(camera_front), std::ref(queue_front), std::ref(global_stop)));
        camera_threads[1] = std::thread(std::bind(&Avm::cameraProcess, this, std::ref(video_back), std::ref(camera_back), std::ref(queue_back), std::ref(global_stop)));
        camera_threads[2] = std::thread(std::bind(&Avm::cameraProcess, this, std::ref(video_left), std::ref(camera_left), std::ref(queue_left), std::ref(global_stop)));
        camera_threads[3] = std::thread(std::bind(&Avm::cameraProcess, this, std::ref(video_right), std::ref(camera_right), std::ref(queue_right), std::ref(global_stop)));
        const auto deal_start_time = std::chrono::system_clock::now();
        std::cout << "AVM Processing..........." << std::endl;
        while (!global_stop.load())
        {
            // 等待所有队列中有数据
            auto wait_start_time = std::chrono::steady_clock::now();
            while (queue_front.size() == 0 || queue_back.size() == 0 || 
                queue_left.size() == 0 || queue_right.size() == 0) {
                        // 检查等待是否超时
                    if (std::chrono::steady_clock::now() - wait_start_time > wait_timeout_duration) {
                        std::cerr << "Wait timeout : " << 5 * 1000 << "ms"  << " avm program stop." << std::endl;
                        global_stop = true;
                        // 唤醒所有等待中线程
                        queue_front.notify_all();
                        queue_back.notify_all();
                        queue_left.notify_all();
                        queue_right.notify_all();
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ms
                    // continue;
            }
            if (global_stop.load()) break;

            // 取出四帧（保证同一批次的帧）
            queue_front.pop(frame_front);
            queue_back.pop(frame_back);
            queue_left.pop(frame_left);
            queue_right.pop(frame_right);

            // 拼接四帧
            bv.add_4frames(frame_front, frame_back, frame_left, frame_right);
            frame_stitched = bv.stitch_all_parts();
            
            #if WHITE_BALANCE
                uv.make_white_blance(frame_stitched, frame_whited);  // Step5: White Balance
                uv.make_luminace_balance_yuv(frame_whited);          // Step6: Luminance Balance
            #endif
            
            stitched_output.write(frame_stitched);
            
            cnt++; 
            // std::cout << "frame num = " << cnt << std::endl;
        }
        const auto deal_end_time = std::chrono::system_clock::now();
        std::cout << "AVM End, Cost Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(deal_end_time - deal_start_time).count() << " ms" << std::endl;
        std::cout << "Total Frame : " << cnt << std::endl;
        // 等待线程结束
        for (auto& t : camera_threads) {
            if(t.joinable()){
                // std::cerr << "Joining thread: " << t.get_id() << std::endl;
                t.join();
            }
        }
        video_front.release();
        video_back.release();
        video_left.release();
        video_right.release();
        stitched_output.release();
        cv::imwrite(stitched_out_path, frame_stitched);
        // cv::imwrite(white_balance_stitched_out_path, frame_whited);
            
    }else{
        std::cerr << "Failed to load videos." << std::endl;
        return;
    }
}
