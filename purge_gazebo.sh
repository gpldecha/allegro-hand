#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'


while ! echo "$PW" | sudo -S -v > /dev/null 2>&1; do
    read -s -p "Enter your sudo password: " PW
    echo
done


remove_gazebo_in_path(){
	sudo find $1 -name '*gazebo*' | while read line; do
		sudo rm -fr $line
	done
}

printf "${GREEN}removing gazebo\n${NC}"
sudo apt-get -q -y remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'
sudo apt-get -q -y autoremove
remove_gazebo_in_path /usr/bin
remove_gazebo_in_path /usr/lib
remove_gazebo_in_path /usr/include
remove_gazebo_in_path /usr/local/

sudo apt autoremove
