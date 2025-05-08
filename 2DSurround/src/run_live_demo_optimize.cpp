#include "FisheyeCameraModel.h"
#include "CalibrateCamera.h"
#include "BirdView.h"

#include "UtilsView.h"
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>

// #define CALIBRATE_CAMERA
#define WHITE_BALANCE 0
std::atomic<bool> global_stop(false);

struct DEVICE
{
    std::string front_ = "front";
    std::string back_ = "back";
    std::string left_ = "left";
    std::string right_ = "right";
};

DEVICE device_;

// 线程安全队列
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mtx;
    std::condition_variable cond;
    const size_t MAX_SIZE = 500;  // 根据内存调整
    std::atomic<bool>& stop_flag;  // 引用全局停止标志
public:
    ThreadSafeQueue(std::atomic<bool>& stop) : stop_flag(stop) {}  // 通过构造函数注入
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        // 等待条件：队列未满 或 需要停止
        cond.wait(lock, [this] { return queue.size() < MAX_SIZE || stop_flag.load(); });
        if (stop_flag.load()) return; 
        queue.push(item);
        cond.notify_one();
    }
    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        if (queue.empty()) return false;
        item = queue.front();
        queue.pop();
        return true;
    }

    void notify_all() {
        cond.notify_all();  // 唤醒所有等待线程
    }
    void wait_and_pop(T& item) {
        std::unique_lock<std::mutex> lock(mtx);
        cond.wait(lock, [this]{ return !queue.empty(); });
        item = queue.front();
        queue.pop();
    }
    size_t size() {
        std::lock_guard<std::mutex> lock(mtx);
        return queue.size();
    }
};


// 摄像头处理
void process_camera(
    cv::VideoCapture& video,
    FisheyeCameraModel& camera,
    ThreadSafeQueue<cv::Mat>& output_queue,
    std::atomic<bool>& stop_flag) {
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


ThreadSafeQueue<cv::Mat> queue_front(global_stop), queue_back(global_stop), queue_left(global_stop), queue_right(global_stop);
int main(int argc, char *argv[])
{
    // Test Instance CalibrateCamera
    #ifdef CALIBRATE_CAMERA
        CalibrateCamera camera0(argv[1], cv::Size(9, 6), cv::Size(10, 10));
        
        camera0.extract_corners();
        camera0.calibrate_process();
        camera0.calc_project_matrix();
        camera0.save_calibrated_yaml();
    #endif
    
    // camera front
    FisheyeCameraModel camera_front(device_.front_);
    camera_front.load_camera_params();
    camera_front.update_undistort_maps();    // Must mapx and mapy for remap function

    cv::VideoCapture video_front("../data/videos/front0810_1280.avi");
    if (!video_front.isOpened())
    {
        std::cerr << "video front is opened failed!" << std::endl;
        return -1;
    }
    
    // camera back
    FisheyeCameraModel camera_back(device_.back_);
    camera_back.load_camera_params();
    camera_back.update_undistort_maps();
    
    cv::VideoCapture video_back("../data/videos/back0810_1280.avi");
    if (!video_back.isOpened())
    {
        std::cerr << "video back is opened failed!" << std::endl;
        return -1;
    }
    
    // camera left
    FisheyeCameraModel camera_left(device_.left_);
    camera_left.load_camera_params();
    camera_left.update_undistort_maps();
    
    cv::VideoCapture video_left("../data/videos/left0810_1280.avi");
    if (!video_left.isOpened())
    {
        std::cerr << "video left is opened failed!" << std::endl;
        return -1;
    }
    
    // camera right
    FisheyeCameraModel camera_right(device_.right_);
    camera_right.load_camera_params();
    camera_right.update_undistort_maps();
    
    cv::VideoCapture video_right("../data/videos/right0810_1280.avi");
    if (!video_right.isOpened())
    {
        std::cerr << "video right is opened failed!" << std::endl;
        return -1;
    }
    
    // car model 
    cv::Mat car_model;
    car_model = cv::imread("../data/images/fisheye/car.png");
    BirdView bv(car_model);
    UtilsView uv;
    
    // stitch 4 frames
    ParamSettings params;
    cv::VideoWriter stitched_output("../data/videos/stitched_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(params.total_w, params.total_h));
    
    std::cout << "camera: " << device_.front_ << ", frame: " << video_front.get(cv::CAP_PROP_FRAME_COUNT) 
              << ", " << "W x H: " << video_front.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
              << video_front.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "camera: " << device_.back_  << ", frame: " << video_back.get(cv::CAP_PROP_FRAME_COUNT) 
              << ", " << "W x H: " << video_back.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
              << video_back.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "camera: " << device_.left_  << ", frame: " << video_left.get(cv::CAP_PROP_FRAME_COUNT) 
              << ", " << "W x H: " << video_left.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
              << video_left.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "camera: " << device_.right_ << ", frame: " << video_right.get(cv::CAP_PROP_FRAME_COUNT) 
              << ", " << "W x H: " << video_right.get(cv::CAP_PROP_FRAME_WIDTH) << " x " 
              << video_right.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    
    // 启动四个摄像头处理线程
    std::thread camera_threads[4];  // front, back, left, right
    camera_threads[0] = std::thread(process_camera, std::ref(video_front), std::ref(camera_front), std::ref(queue_front), std::ref(global_stop));
    camera_threads[1] = std::thread(process_camera, std::ref(video_back), std::ref(camera_back), std::ref(queue_back), std::ref(global_stop));
    camera_threads[2] = std::thread(process_camera, std::ref(video_left), std::ref(camera_left), std::ref(queue_left), std::ref(global_stop));
    camera_threads[3] = std::thread(process_camera, std::ref(video_right), std::ref(camera_right), std::ref(queue_right), std::ref(global_stop));

    // 主循环：从队列中获取四帧并拼接
    cv::Mat frame_front, frame_back, frame_left, frame_right;
    cv::Mat frame_stitched, frame_whited;

    long cnt = 0;
    const std::chrono::seconds wait_timeout_duration(5);  // 5秒等待超时
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
            std::cerr << "Joining thread: " << t.get_id() << std::endl;
            t.join();
        }
    }
    
    video_front.release();
    video_back.release();
    video_left.release();
    video_right.release();
    stitched_output.release();
    
    cv::imwrite("../data/images/fisheye/stitch_result_0.png", frame_stitched);
    // cv::imwrite("../data/images/fisheye/frame_white_result_0.png", frame_whited);
    
    return 0;
}
