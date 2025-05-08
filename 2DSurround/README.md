# ################################## 360view(2dsurround)

## 项目介绍
四个鱼眼摄像头拼接成车辆全景图,原理参考：https://blog.csdn.net/weixin_48726650/article/details/107411925

## 软硬件配置
1. 硬件配置：4 x usb摄像头
2. 软硬件配置：
- 运行环境：Ubuntu 20.04
- OpenCV：4.5.5
- g++ 9.4.0
- cmake 3.16.3
- yaml-cpp：0.6.2

## 编译
mkdir build && cd build && cmake .. && make -j8

# dokcer 项目镜像部署

1. docker build
- ceate image: docker build -t 2dsurround-image .
- save image： docker save -o 2dsurround-image.tar 2dsurround-image
- push image： docker push 2dsurround-image (docker login first) 或将 2dsurround-image.tar 上传到服务器
- load image： docker load -i 2dsurround-image.tar
- creat docker： docker run -d -p 8080:8080 --name 360view 2dsurround-image:latest