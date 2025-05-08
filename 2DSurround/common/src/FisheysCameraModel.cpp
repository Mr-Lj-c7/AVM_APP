#include "FisheyeCameraModel.h"

FisheyeCameraModel::FisheyeCameraModel(const std::string &camera_name)
{
    this->camera_name = camera_name;
    
    this->scale_xy = cv::Vec2f(1.0, 1.0);
    this->shift_xy = cv::Vec2f(0.0, 0.0);
    
    this->project_shape = params.project_shapes[camera_name];
}

// std::string FisheyeCameraModel::get_camera_name()
// {
//     return camera_name;
// }

void FisheyeCameraModel::load_camera_params()
{
    camera_file = "../config/fisheye/" + camera_name + ".yaml";
    // camera_file = "./config/fisheye/" + camera_name + ".yaml";  // docker
    
    cv::FileStorage fs;
    fs.open(camera_file, cv::FileStorage::READ);
    
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["resolution"] >> resolution;
    fs["project_matrix"] >> project_matrix;
    fs["scale_xy"] >> scale_xy;
    fs["shift_xy"] >> shift_xy;
    
    fs.release();
}

// 更新去畸变映射
void FisheyeCameraModel::update_undistort_maps()
{
    // camera inner params
    cv::Mat new_matrix = camera_matrix.clone();
    new_matrix.at<double>(0, 0) *= scale_xy.at<float>(0, 0);
    new_matrix.at<double>(1, 1) *= scale_xy.at<float>(1, 0);
    new_matrix.at<double>(0, 2) += shift_xy.at<float>(0, 0);
    new_matrix.at<double>(1, 2) += shift_xy.at<float>(1, 0);
    
    // Resolution
    int width = resolution.at<int>(0, 0), height = resolution.at<int>(1, 0);
    
    if (map1.empty() || map2.empty()){
        cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat::eye(cv::Size(3, 3), CV_32F), new_matrix, cv::Size(width, height), CV_32FC1, map1, map2);
    }
    // cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat::eye(cv::Size(3, 3), CV_32F), new_matrix, cv::Size(width, height), CV_32FC1, map1, map2);
}

// 图像的缩放和平移
void FisheyeCameraModel::set_scale_and_shift(cv::Vec2f scale_xy, cv::Vec2f shift_xy)
{
    this->scale_xy = scale_xy;
    this->shift_xy = shift_xy;
    
    update_undistort_maps();
}

// 去畸变
void FisheyeCameraModel::undistort_remap(const cv::Mat &image, cv::Mat &result)
{
    if (!map1.data || !map2.data)
    {
        std::cerr << "map1 or map2 is empty!" << std::endl;
        return ;
    }
    cv::remap(image, result, map1, map2, cv::INTER_LINEAR);
}

void FisheyeCameraModel::project_warp_perspective(const cv::Mat &image, cv::Mat &result)
{
    // 透视变换
    cv::warpPerspective(image, result, project_matrix, cv::Size(project_shape[0], project_shape[1]));
    
    if ("back" == camera_name)
    {
        // cv::flip(result, result, -1);   // 垂直+水平翻转 = 旋转180°
        cv::rotate(result, result, cv::ROTATE_180);
    }
    else if ("left" == camera_name)     // 逆时针旋转90°
    {
        cv::rotate(result, result, cv::ROTATE_90_COUNTERCLOCKWISE);   
    }
    else if ("right" == camera_name)    // 顺时针旋转90°
    {  
        cv::rotate(result, result, cv::ROTATE_90_CLOCKWISE); 
    }
    else 
    {
        ;   // front camera: do nothing.
    }
    
}

void FisheyeCameraModel::save_data()
{
    cv::FileStorage fs;
    fs.open(this->camera_file, cv::FileStorage::WRITE);
    
    fs << "camera_matrix" << this->camera_matrix;
    fs << "dist_coeffs" << this->dist_coeffs;
    fs << "resolution" << this->resolution;
    fs << "project_matrix" << this->project_matrix;
    fs << "scale_xy" << this->scale_xy;
    fs << "shift_xy" << this->shift_xy;
    
    fs.release();
}
