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
		sudo apt-get -q -y install $1 1> /dev/null
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}

remove_gazebo_in_path(){
	sudo find $1 -name '*gazebo*' | while read line; do
		sudo rm -fr $line
	done
}

install_gazebo_ros_pkgs(){
  if [[ ! -d ../gazebo_ros_pkgs  ]];
  then
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git ../gazebo_ros_pkgs
    cd ../gazebo_ros_pkgs
    git checkout kinetic-devel
    cd ../allegro-hand
  else
      printf "${GREEN}gazebo_ros_pkgs ... ok\n${NC}"
  fi
}


install_gazebo_source(){
	# remove old version of gazebo both sim and dpkg
  remove_gazebo_in_path /usr/bin
	remove_gazebo_in_path /usr/lib
	remove_gazebo_in_path /usr/include
  remove_gazebo_in_path /usr/local

	printf "${GREEN}adding osrfoundation to sources gazebo\n${NC}"
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	sudo apt-get -q -y update 1> /dev/null

	install_library ros-${ROS_DISTRO}-dartsim
	install_library libsdformat6
	install_library libsdformat6-dev
  install_library libignition-math4-dev
  install_library libignition-transport4-dev
  install_library libignition-msgs-dev

	install_library mercurial

	# clone gazebo
	printf "${GREEN}cloning gazebo\n${NC}"
	hg clone https://bitbucket.org/osrf/gazebo/branch/gazebo9 /tmp/gazebo9
	cd /tmp/gazebo9
	printf "${GREEN}checkout gazebo9.2.0\n${NC}"
	hg checkout gazebo9_9.2.0
	mkdir build

	cd build
	cmake -DCMAKE_BUILD_TYPE=Release ../
	# build gazebo
	make -j8
	sudo -S make install <<< "$password"
	sudo -S ldconfig <<< "$password"
}

install_gazebo_source
