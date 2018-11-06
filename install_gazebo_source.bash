#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'


ROS_DISTRO=$(ls /opt/ros/) 
INSTALL_GAZEBO_FROM_SOURCE=true
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

remove_gazebo_in_path(){
	sudo find $1 -name '*gazebo*' | while read line; do
		echo "removing file '$line'" 
		sudo rm -fr $line
	done
}

remove_gazebo_dpkg(){
	remove_gazebo_in_path /usr/bin
	remove_gazebo_in_path /usr/lib
	remove_gazebo_in_path /usr/include
}

remove_gazebo_source(){
	remove_gazebo_in_path /usr/local/
}

purge_gazebo(){
	printf "${GREEN}removing gazebo\n${NC}"
	apt-get -q --yes --force-yes remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' > /dev/null
	apt-get -q --yes --force-yes autoremove  > /dev/null
	remove_gazebo_dpk
	remove_gazebo_source
}

install_gazebo_source(){
	# remove old version of gazebo both sim and dpkg
	purge_gazebo

	# gazebo ros stuff
	install_library ros-${ROS_DISTRO}-gazebo9-dev	
	install_library ros-${ROS_DISTRO}-gazebo9-ros-pkgs
	install_library ros-${ROS_DISTRO}-gazebo9-ros-control
	
	# purge gazebo dpkg (necesary to trick ros)
	remove_gazebo_dpkg
	
	# add osr foundation to source
	printf "${GREEN}adding osrfoundation to sources gazebo\n${NC}"
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	apt-get -q --yes --force-yes update > /dev/null

	# install gazebo dependencies
	printf "${GREEN}install gazebo dependencies\n${NC}"
	
	wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
	ROS_DISTRO=dummy . /tmp/dependencies.sh
	apt-get -q --yes --force-yes install $(sed 's:\\ ::g' <<< $BASE_DEPENDENCIES) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPENDENCIES)
	
	install_library ros-${ROS_DISTRO}-dartsim
	install_library libsdformat6
	install_library libsdformat6-dev
	install_library libignition-msgs-dev
	install_library libignition-transport4-dev 
	
	printf "${GREEN}cloning gazebo\n${NC}"
	# clone gazebo
	hg clone https://bitbucket.org/osrf/gazebo/branch/gazebo9 /tmp/gazebo9
	cd /tmp/gazebo9
	printf "${GREEN}checkout gazebo9.4.1\n${NC}"
	hg checkout gazebo9_9.4.1
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release
	# build gazebo
	make -j8
	sudo make install
	sudo ldconfig
}

install_gazebo_source