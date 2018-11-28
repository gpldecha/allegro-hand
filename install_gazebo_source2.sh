#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'


while ! echo "$PW" | sudo -S -v > /dev/null 2>&1; do
    read -s -p "Enter your sudo password: " PW
    echo
done

ROS_DISTRO=$(ls /opt/ros/) 


install_library(){
	LIBRARY=$(dpkg -S $1 2> /dev/null )
	if [[ -z $LIBRARY  ]]; 
	then
		printf "${GREEN}installing $1\n${NC}"
		sudo apt-get -q -y install $1 #1> /dev/null
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}

remove_gazebo_in_path(){
	sudo find $1 -name '*gazebo*' | while read line; do
		sudo rm -fr $line
	done
}

remove_apt_get_install_gazebo(){
	remove_gazebo_in_path /usr/bin
	remove_gazebo_in_path /usr/include
	remove_gazebo_in_path /usr/lib
	remove_gazebo_in_path /usr/share
}

purge_gazebo(){
	printf "${GREEN}removing gazebo\n${NC}"
	sudo apt-get -q -y remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' 1> /dev/null
	sudo apt-get -q -y autoremove  #1> /dev/null
	remove_gazebo_in_path /usr/local/include
	remove_gazebo_in_path /usr/local/lib
	remove_apt_get_install_gazebo
}

install_gazebo_source(){
	# remove old version of gazebo both sim and dpkg
	purge_gazebo

	printf "${GREEN}adding osrfoundation to sources gazebo\n${NC}"
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	sudo apt-get -q -y update 1> /dev/null

	# gazebo ros stuff
	install_library ros-${ROS_DISTRO}-gazebo9-dev
	install_library ros-${ROS_DISTRO}-gazebo9-ros-pkgs
	install_library ros-${ROS_DISTRO}-gazebo9-ros-control

	# purge gazebo dpkg (necesary to trick ros)
	remove_apt_get_install_gazebo

	install_library ros-${ROS_DISTRO}-dartsim
	install_library libsdformat6
	install_library libsdformat6-dev
	install_library libignition-msgs-dev
	install_library libignition-math4-dev
	install_library libignition-transport4-dev


	#install_library libfreeimage-dev
	#install_library libxml2-dev
	#install_library libprotobuf-dev
	#install_library libprotoc-dev
	#install_library libsimbody-dev
	#install_library libignition-common-dev
	#install_library libignition-fuel-tools-dev

	install_library mercurial

	# clone gazebo
	printf "${GREEN}cloning gazebo\n${NC}"
	hg clone https://bitbucket.org/osrf/gazebo/branch/gazebo9 /tmp/gazebo9
	cd /tmp/gazebo9
	printf "${GREEN}checkout $1\n${NC}"
	hg checkout gazebo9_$1
	mkdir build

	cd build
	cmake -DCMAKE_BUILD_TYPE=Release ../
	# build gazebo
	make -j8
	sudo -S make install <<< "$password" 1> /dev/null
	sudo -S ldconfig <<< "$password" 1> /dev/null
}

install_library ros-${ROS_DISTRO}-gazebo9-dev
install_library ros-${ROS_DISTRO}-gazebo9-ros-pkgs
install_library ros-${ROS_DISTRO}-gazebo9-ros-control

install_library ros-${ROS_DISTRO}-dartsim
install_library libsdformat6
install_library libsdformat6-dev
install_library libignition-msgs-dev
install_library libignition-math4-dev
install_library libignition-transport4-dev
