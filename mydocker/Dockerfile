# FROM ros:melodic
# use command below so you can use rviz
FROM osrf/ros:melodic-desktop-full

COPY sources.list /etc/apt/sources.list

ENV HOME /root
ENV CERES_VERSION="2.1.0"
# ENV PCL_VERSION="1.8.0"
# ENV CATKIN_WS=/root/catkin_ws


RUN DEBIAN_FRONTEND=noninteractive apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	libgflags-dev \
	libgoogle-glog-dev \
	libatlas-base-dev \
	libeigen3-dev \
	libsuitesparse-dev \
	git \
	vim \
	wget \
	unzip \
	libboost-all-dev \ 
	apt-utils \
  curl \
	ros-${ROS_DISTRO}-cv-bridge \
	ros-${ROS_DISTRO}-image-transport \
	ros-${ROS_DISTRO}-message-filters \
	ros-${ROS_DISTRO}-tf \
	ros-${ROS_DISTRO}-pcl-conversions \
	ros-${ROS_DISTRO}-pcl-ros \
	ros-${ROS_DISTRO}-eigen-conversions \
	ros-${ROS_DISTRO}-geographic-msgs \
	ros-${ROS_DISTRO}-geodesy \
	software-properties-common \
	libyaml-cpp-dev \
#	libopencv-dev \
#	apt-file \
  && DEBIAN_FRONTEND=noninteractive apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get autoclean \
  && DEBIAN_FRONTEND=noninteractive apt-get autoremove \
  && DEBIAN_FRONTEND=noninteractive rm -rf /var/lib/apt/lists/*

# install ceres
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi \
  && mkdir -p /libdir \
  && cd /libdir \
  && git clone https://ceres-solver.googlesource.com/ceres-solver \
  && cd ./ceres-solver \
  && git checkout ${CERES_VERSION} \
  && cmake . \
  && make -j${USE_PROC} install \
  && cd .. \
  && rm -r ./ceres-solver \
  && cd / \
  && rm -r /libdir

# install gtsam
RUN DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:borglab/gtsam-release-4.0 \
  && DEBIAN_FRONTEND=noninteractive apt-get update \ 
  && DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends install libgtsam-dev libgtsam-unstable-dev \
  && DEBIAN_FRONTEND=noninteractive apt-get autoclean \
  && DEBIAN_FRONTEND=noninteractive rm -rf /var/lib/apt/lists/*

# dependencies used by SmartCar, maybe occur errors in noetic
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
	ros-${ROS_DISTRO}-serial \
	ros-${ROS_DISTRO}-qt-build \
	ros-${ROS_DISTRO}-roslint \
	ros-${ROS_DISTRO}-jsk-recognition-msgs \
	ros-${ROS_DISTRO}-camera-info-manager \
	ros-${ROS_DISTRO}-diagnostic-updater \
	ros-${ROS_DISTRO}-image-geometry \
	ros-${ROS_DISTRO}-angles \
	freeglut3-dev \
	libgeographic-dev \
	libpcap-dev \
	python-pip \
  python3-pip \
  && DEBIAN_FRONTEND=noninteractive apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get autoclean \
  && DEBIAN_FRONTEND=noninteractive apt-get autoremove \
  && DEBIAN_FRONTEND=noninteractive rm -rf /var/lib/apt/lists/* \
  && pip install apscheduler -i https://pypi.tuna.tsinghua.edu.cn/simple --no-cache-dir\
  && pip install flask -i https://pypi.tuna.tsinghua.edu.cn/simple --no-cache-dir\
  && pip install ConfigParser -i https://pypi.tuna.tsinghua.edu.cn/simple --no-cache-dir\
  && pip install requests -i https://pypi.tuna.tsinghua.edu.cn/simple --no-cache-dir\
  && pip install cmake==3.16.6 --no-cache-dir\
  && pip3 install numpy --no-cache-dir\
  && pip3 install matplotlib --no-cache-dir

# install g2o  -- cmake 3.14 or higher required
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi \
  && mkdir -p /libdir \
  && cd /libdir \
  && git clone https://github.com/RainerKuemmerle/g2o.git \
  && cd ./g2o \
  && cmake -DG2O_USE_VENDORED_CERES=OFF . \
  && make -j${USE_PROC} install \
  && cd .. \
  && rm -r ./g2o \
  && cd / \
  && rm -r /libdir


# 解决kdtree相关问题
RUN sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak \
  && sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak \
  && sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h \
  && sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h \
  && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib \
  && bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"


WORKDIR /
