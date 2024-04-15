FROM ryuichiueda/ubuntu18.04-ros-image:latest

ENV PATH=/usr/local/bin:${PATH}

# 构建 ros层
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 && \
    apt update && \
    apt install ros-melodic-desktop-full -y && \
    apt install ros-melodic-pcl-ros ros-melodic-velodyne-msgs -y && \
    apt install libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libeigen3-dev libsuitesparse-dev -y 

# 构建其他依赖
RUN apt install libomp-dev -y && \
    sudo apt install build-essential libgl1-mesa-dev -y && \
    sudo apt install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev -y&& \
    sudo apt install libglfw3-dev libglfw3  -y

# 构建ceres 层
WORKDIR /app

COPY ceres ceres

WORKDIR /app/ceres

RUN mkdir ceres-bin && cd ceres-bin && \
    cmake .. && \
    make -j24 && \
    make install


# 修复ROS的一些bug
RUN mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak && \
    mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak && \
    ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h && \
    ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

ENV USER=root
ENV PYTHONPATH=$PYTHONPATH:/opt/ros/melodic/lib/python2.7/dist-packages
ENV PATH=/opt/ros/melodic/bin:/opt/ros/melodic/bin:${PATH}
ENV CMAKE_PREFIX_PATH=/catkin_ws/devel:/opt/ros/melodic:${CMAKE_PREFIX_PATH}

WORKDIR /my-app/src
# # 添加示例程序
RUN  catkin_init_workspace && \
     git clone https://github.com/APRIL-ZJU/lidar_IMU_calib && \
     wstool init && \
     wstool merge lidar_IMU_calib/depend_pack.rosinstall && \
     wstool update && \
     cd lidar_IMU_calib/ && \
    ./build_submodules.sh

WORKDIR /my-app
RUN catkin_make 

CMD source ./devel/setup.bash;./src/lidar_IMU_calib/calib.sh;/bin/bash

