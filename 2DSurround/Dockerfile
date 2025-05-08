# 使用官方的ubuntu20.04基础镜像
FROM ubuntu:20.04

# 设置工作目录
WORKDIR /app

#定义时区参数
ENV TZ=Asia/Shanghai
#设置时区
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' > /etc/timezone

# 安装必要的依赖
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    g++ \
    libopencv-dev=4.2.0+dfsg-5 \
    libyaml-cpp-dev 

# 复制项目文件到镜像中
COPY . /app

# 构建项目
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make

# 检查文件是否存在
RUN if [ ! -f /app/bin/avm_app_demo ]; then echo "File /app/bin/avm_app_demo not found"; exit 1; fi

# 设置可执行文件权限
RUN chmod =x /app/bin/avm_app_demo

# 设置容器启动命令
CMD ["./bin/avm_app_demo"]

# 暴露端口
# EXPOSE 8080