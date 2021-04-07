FROM nvidia/vulkan:1.1.121

LABEL authors="Allessandro Fornasier<allessandro.fornasier@aau.at> \
Martin Scheiber<martin.scheiber@aau.at>"
LABEL version="1.0.0"
LABEL description="Docker container for the vinseval_cws."


# install other packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    autotools-dev \
    bc \
    build-essential \
    aptitude \
    autoconf \
    curl \
    htop \
    locales \
    lsb-release \
    nano \
    net-tools \
    openssh-client \
    rsync \
    software-properties-common \
    unzip \
    usbutils \
    libsdl1.2-dev \
    ruby \
    && rm -rf /var/lib/apt/lists/*

########################################
## ROS
########################################

# install ROS
## setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
        apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
## setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
ENV ROS_DISTRO melodic
RUN rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-mav-msgs \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-stereo-image-proc \
    ros-$ROS_DISTRO-rospy \
    ros-$ROS_DISTRO-eigen-conversions \
    && rm -rf /var/lib/apt/lists/*

# catkin build and wstool
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    python-wstool \
    python-pandas \
    && rm -rf /var/lib/apt/lists/*

########################################
## Dependencies for VINSEval Estimators
########################################

# Ceres (all estimators)
WORKDIR /opt/ceres/
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*
RUN wget ceres-solver.org/ceres-solver-2.0.0.tar.gz
RUN tar -zxf ceres-solver-2.0.0.tar.gz && mkdir ceres-bin
WORKDIR /opt/ceres/ceres-bin/
RUN cmake ../ceres-solver-2.0.0
RUN make -j4
RUN make install

# OpenCV (all estimators)
RUN apt-get update && apt-get install -y \
    python-numpy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/opencv/
RUN git clone --branch 3.4.13 https://github.com/opencv/opencv/
RUN git clone --branch 3.4.13 https://github.com/opencv/opencv_contrib/
RUN mkdir opencv/build/
WORKDIR /opt/opencv/opencv/build/
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
RUN make -j4
RUN make install

# Pangolin (LARVIO)
#RUN apt-get update && apt-get install -y \
#    libgl1-mesa-dev \
#    libglew-dev \
#    pkg-config \
#    libwayland-dev \
#    libxkbcommon-dev \
#    wayland-protocols \
#    && rm -rf /var/lib/apt/lists/*
#WORKDIR /opt/pangolin
#RUN git clone https://github.com/stevenlovegrove/Pangolin.git
#WORKDIR /opt/pangolin/Pangolin/build
#RUN cmake ..
#RUN cmake --build .

# kindr (ROVIO)
#WORKDIR /opt/kindr
#RUN git clone https://github.com/ethz-asl/kindr.git
#WORKDIR /opt/kindr/kindr
#RUN git checkout 22a19ddd
#WORKDIR /opt/kindr/kindr/build
#RUN cmake ..
#RUN make install

########################################
## VINS Eval
########################################

# Prequesites to run Unity3D executables
RUN apt-get update && apt-get install -y \
    libzmqpp-dev \
    libeigen3-dev \
    libvulkan1 \
    vulkan-utils gdb \
    && rm -rf /var/lib/apt/lists/*

# Fixes, for upstairs
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-tf-conversions \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros \
    freeglut3-dev \
    libglew-dev \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Setup VINSEval workspace
ENV CATKIN_WS=vinseval_cws
WORKDIR /root/
RUN mkdir -p $CATKIN_WS/src
COPY ./src $CATKIN_WS/src/

# Install Python requirements for VINSEval tools
WORKDIR $CATKIN_WS/src/eval_tools/
RUN rm -rf venv

#RUN /bin/bash -c "apt update && ./setup-env.sh"
RUN apt update 
RUN ./setup-env.sh

#WORKDIR /opt/kindr_ws/
#RUN mkdir src && ln -s /opt/kindr/kindr src/
#RUN /bin/bash -c "catkin init; \
#                  catkin config --extend /opt/ros/$ROS_DISTRO; \
#                  catkin config -j 4 -l 4 --cmake-args \
#                  -DCMAKE_BUILD_TYPE=Release"
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; \
#                  catkin build -w /opt/kindr_ws kindr"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
WORKDIR /root/$CATKIN_WS/
RUN /bin/bash -c "catkin init; \
#                  catkin config --extend /opt/kindr_ws/devel; \
                  catkin config --extend /opt/ros/$ROS_DISTRO; \
                  catkin config -j 4 -l 4 --cmake-args \
                  -DFLIGHTGOGGLES_DOWNLOAD_BINARY=OFF \
                  -DCMAKE_BUILD_TYPE=Release"

# Link kindr for rovio
# RUN ln -s /opt/kindr/kindr src/
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build -w /root/$CATKIN_WS/ kindr"

# Build workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN catkin build

# Update
RUN apt update

RUN echo 'source /root/$CATKIN_WS/devel/setup.bash' >> /root/.bashrc
RUN echo 'export PYTHONPATH=$PYTHONPATH:/root/$CATKIN_WS/src/eval_tools' >> /root/.bashrc
RUN echo 'source /root/$CATKIN_WS/src/eval_tools/python-venv/env/bin/activate' >> /root/.bashrc

########################################
## VINSEval Scripts & Trajectorys
########################################

WORKDIR /root/
RUN mkdir -p $CATKIN_WS/scripts
COPY ./script/docker /root/$CATKIN_WS/scripts/
COPY ./script/switch_bag_time.py /root/$CATKIN_WS/scripts/switch_bag_time.py
COPY ./script/Odometry2PoseWithCovarianceStamped.py /root/$CATKIN_WS/scripts/Odometry2PoseWithCovarianceStamped.py
WORKDIR /root/$CATKIN_WS/scripts
RUN chmod +x ./*.sh ./*.py

RUN mkdir -p $CATKIN_WS/trajectories
COPY ./trajectories /root/$CATKIN_WS/trajectories

# TODO: this needs to be downloaded from official page
RUN mkdir -p $CATKIN_WS/devel/lib/flightgoggles
COPY ./devel/lib/flightgoggles /root/$CATKIN_WS/devel/lib/flightgoggles/

########################################
## Display access
########################################


RUN apt-get update && apt-get install -y \
    xauth \
    -qqy \
    xterm \
    x11-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display
