#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

ROS_DISTRO=$(ls /opt/ros/) 
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

install_pcan_basic(){
	cd ${DIR}/pcan_basic/pcanbasic/
	make 
	make install
	cd ../../
}

install_library(){
	LIBRARY=$(dpkg -S $1 )
	if [[ -z $LIBRARY  ]]; 
	then
		printf "${GREEN}installing $1\n${NC}"
		apt-get -qq --yes --force-yes install $1
	else
		printf "${GREEN}$1 ... ok\n${NC}"
	fi
}


install_pcan_basic
