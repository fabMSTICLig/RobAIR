#!/bin/bash

DisplayIntro()
{
	echo "______      _      ___  ___________"
	echo "| ___ \    | |    / _ \|_   _| ___ \ "
	echo "| |_/ /___ | |__ / /_\ \ | | | |_/ /"
	echo "|    // _ \|  _ \|  _  | | | |    /"
	echo "| |\ \ (_) | |_) | | | |_| |_| |\ \ "
	echo "\_| \_\___/|_.__/\_| |_/\___/\_| \_|"
	echo "===================================="
	echo           "DOCKING SETUP"
	echo "===================================="
	echo "Welcom !"
	echo "This setup is intended to simplify docking's installation"
}

DisplayMenu ()
{
	echo "===================================="
	echo "0) Exit"
	echo "1) Install ALL"
	echo "2) └───Install ROS"
	echo "3) 	└───Prerequisites"
	echo "4) 	└───Installation"
	echo "5) 	└───Building"
	echo "6) 	└───Update"
	echo "7) └───Install OpenCV"
	echo "8) 	└───Prerequisites"
	echo "9) 	└───Installation"
	echo "10) 	└───Building"
	echo
}

#######
# ALL #
#######

InstallALL()
{
	InstallROS
	InstallOpenCV
}

#######
# ROS #
#######

InstallROS()
{
	InstallROSPrerequisites
	InstallROSInstallation
	InstallROSBuilding
	InstallROSUpdate
}

InstallROSPrerequisites()
{
	echo -e "\e[93m===================================="
	echo "ROS - Prerequisites - Start"
	echo -e "====================================\e[0m"

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	sudo apt-get update
	sudo apt-get upgrade
	sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
	sudo rosdep init
	rosdep update

	echo -e "\e[93m===================================="
	echo "ROS - Prerequisites - End"
	echo -e "====================================\e[0m"
}

InstallROSInstallation()
{
	echo -e "\e[93m===================================="

	echo "ROS - Installation - Start"
	echo -e "====================================\e[0m"

	mkdir -p ~/ros_catkin_ws
	cd ~/ros_catkin_ws
    rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
    wstool init src kinetic-ros_comm-wet.rosinstall
    mkdir -p ~/ros_catkin_ws/external_src
	cd ~/ros_catkin_ws/external_src
	wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
	unzip assimp-3.1.1_no_test_models.zip
	cd assimp-3.1.1
	cmake .
	make
	sudo make install
	cd ~/ros_catkin_ws
	rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie

	echo -e "\e[93m===================================="
	echo "ROS - Installation - End"
	echo -e "====================================\e[0m"
}

InstallROSBuilding()
{
	echo -e "\e[93m===================================="
	echo "ROS - Building - Start"
	echo -e "====================================\e[0m"

	cd ~/ros_catkin_ws
	sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
	sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2
	source /opt/ros/kinetic/setup.bash
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

	echo -e "\e[93m===================================="
	echo "ROS - Building - End"
	echo -e "====================================\e[0m"
}

InstallROSUpdate()
{
	echo -e "\e[93m===================================="
	echo "ROS - Update - Start"
	echo -e "====================================\e[0m"
	
	cd ~/ros_catkin_ws
	rosinstall_generator ros_comm ros_control joystick_drivers --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall
	wstool merge -t src kinetic-custom_ros.rosinstall
	wstool update -t src
	rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie
	sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

	echo -e "\e[93m===================================="
	echo "ROS - Update - End"
	echo -e "====================================\e[0m"
}

##########
# OpenCV #
##########

InstallOpenCV()
{
	InstallOpenCVPrerequisites
	InstallOpenCVInstallation
	InstallOpenCVBuilding
}

InstallOpenCVPrerequisites()
{
	echo -e "\e[93m===================================="
	echo "OpenCV - Prerequisites - Start"
	echo -e "====================================\e[0m"
	
	sudo apt-get update
	sudo apt-get upgrade
	sudo apt-get install -y build-essential cmake pkg-config
	sudo apt-get install -y libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
	sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
	sudo apt-get install -y libxvidcore-dev libx264-dev
	sudo apt-get install -y libgtk2.0-dev
	sudo apt-get install -y libatlas-base-dev gfortran
	sudo apt-get install -y python2.7-dev python3-dev

	echo -e "\e[93m===================================="
	echo "OpenCV - Prerequisites - End"
	echo -e "====================================\e[0m"
}

InstallOpenCVInstallation()
{
	echo -e "\e[93m===================================="
	echo "OpenCV - Installation - Start"
	echo -e "====================================\e[0m"
	
	cd ~
	git clone https://github.com/opencv/opencv.git
	git clone https://github.com/opencv/opencv_contrib.git
	pip install numpy

	echo -e "\e[93m===================================="
	echo "OpenCV - Installation - End"
	echo -e "====================================\e[0m"
}

InstallOpenCVBuilding()
{
	echo -e "\e[93m===================================="
	echo "OpenCV - Building - Start"
	echo -e "====================================\e[0m"
	

	cd ~/opencv
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=/usr/local \
	    -D INSTALL_PYTHON_EXAMPLES=ON \
	    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	    -D BUILD_EXAMPLES=ON ..
	 
	make -j4
	sudo make install
	sudo ldconfig

	echo -e "\e[93m===================================="
	echo "OpenCV - Building - End"
	echo -e "====================================\e[0m"
}

########
# Main #
########

choice=""

DisplayIntro

while [[ $choice != "0" ]]; do

	DisplayMenu

	read -p 'Select your option : ' choice

	case $choice in
		"0" )	exit 0;;
		"1" )	InstallALL;;
		"2" )	InstallROS;;
		"3" )	InstallROSPrerequisites;;
		"4" )	InstallROSInstallation;;
		"5" )	InstallROSBuilding;;
		"6" )	InstallROSUpdate;;
		"7" )	InstallOpenCV;;
		"8" )	InstallOpenCVPrerequisites;;
		"9" )	InstallOpenCVInstallation;;
		"10" ) 	InstallOpenCVBuilding;;
		* ) 	echo "The selected option is not valide";;
	esac

done
