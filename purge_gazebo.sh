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

purge_gazebo