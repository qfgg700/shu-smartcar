#!/bin/bash
echo ' ' | sudo -S chmod 777 /dev/ttyUSB*
echo ' ' | sudo -S chmod a+rw /dev/input/js*
source devel/setup.bash
roslaunch starter all_go.launch

read -p "按回车键返回"
