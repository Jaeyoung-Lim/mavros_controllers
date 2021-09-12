#!/bin/bash

set -e

echo "This script will configure your system"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

PACKAGE_PATH=/home/px4vision/catkin_ws/src/mavros_controllers

# Configure systemd service
echo "Copy systemd service"
cp -vf $PACKAGE_PATH/systemd/mavros_controllers.service /etc/systemd/system/

systemctl enable mavros_controllers.service
