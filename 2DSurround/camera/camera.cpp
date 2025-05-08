#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 打开默认摄像头（通常是0）
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cout << "无法打开摄像头" << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (true) {
        // 读取一帧画面
        cap >> frame;

        if (frame.empty()) {
            std::cout << "无法接收画面（流结束？）" << std::endl;
            break;
        }

        // 显示画面
        cv::imshow("摄像头画面", frame);

        // 按下 'q' 键退出循环
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 释放摄像头并关闭窗口
    cap.release();
    cv::destroyAllWindows();

    return 0;
}