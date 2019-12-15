# TODO: Not finished yet!

FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04

RUN apt update && apt install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && apt install -y \
    git \
    wget \
    libzmq3-dev \
    libnvinfer6 \
    libnvonnxparsers6 \
    libnvparsers6 \
    libnvinfer-plugin6 \
    ros-kinetic-desktop-full \
    ros-kinetic-jsk-recognition \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool build-essential
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc
# TODO: download address
# https://developer.nvidia.com/compute/machine-learning/tensorrt/secure/6.0/GA_6.0.1.5/local_repos/nv-tensorrt-repo-ubuntu1604-cuda10.0-trt6.0.1.5-ga-20190913_1-1_amd64.deb

RUN wget https://yun.yusanshi.com/nv-tensorrt-repo-ubuntu1604-cuda10.0-trt6.0.1.5-ga-20190913_1-1_amd64.deb \
    && dpkg -i nv-tensorrt-repo-ubuntu1604-cuda10.0-trt6.0.1.5-ga-20190913_1-1_amd64.deb \
    && apt-key add /var/nv-tensorrt-repo-cuda10.0-trt6.0.1.5-ga-20190913/7fa2af80.pub \
    && apt update \
    && apt install aptitude \
    && aptitude install tensorrt
# TODO: aptitude?
RUN git clone git@github.com:yusanshi/ros_package.git --recursive \
    && cd ros_package
RUN AUTOWARE_COMPILE_WITH_CUDA=1 catkin_make --force-cmake --only-pkg-with-deps image_detection detected_objects_visualizer pixel_cloud_fusion range_vision_fusion lidar_point_pillars
RUN echo "source ~/ros_package/devel/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc

# TODO: Not finished yet!