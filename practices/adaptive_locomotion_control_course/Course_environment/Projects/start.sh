#! /bin/bash


{
gnome-terminal -t "Start_Ros" -x bash -c "roscore;exec bash"
}&


sleep 1s
{
gnome-terminal -t "Start_coppelia" -x bash -c "cd ../../CoppeliaSim_Edu_V4_0_0_Ubuntu18_04;./coppeliaSim.sh ../Course_environment/utils/CoppeliaSim_simulation/slalom/slalom.ttt;exec bash"
}&


