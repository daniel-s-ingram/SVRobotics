#!/usr/bin/env bash

wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
pip install --user scipy

#Install and configure ROS Kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get -y install python-catkin-tools
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
catkin build
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Install OpenCV
sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install libgtk-3-dev
sudo apt-get -y install libatlas-base-dev gfortran
sudo apt-get -y install python2.7-dev python3.5-dev
cd ~
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.1.0.zip
unzip opencv.zip
mv opencv-3.1.0 opencv
rm opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.1.0.zip
unzip opencv_contrib.zip
mv opencv_contrib-3.1.0 opencv_contrib
rm opencv_contrib.zip
mv opencv_contrib opencv
cd opencv
mkdir build
cd build
echo "find_package(HDF5)" >> ~/opencv/modules/python/common.cmake
echo "include_directories(\${HDF5_INCLUDE_DIRS})" >> ~/opencv/modules/python/common.cmake
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig

#Install misc
pip install --user "picamera[array]"
sudo apt-get -y install arduino
sudo apt-get -y install terminator