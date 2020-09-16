                                            LILIBOT

1) Readme

2) Framework

The control code was put on controllers folder,and the robots folder is the interface code for a real robot. The Vrep simulation model was put in vrep_simulation folder

3) The steps to implement this project on your PC:

3.0) Install VREP
3.1) Install ROS kinetic
3.2) Install git
3.3) mkdir -p ~/workspace/stbot/ && cd ~/workspace/stbot/
3.4) git clone https://gitlab.com/neutron-nuaa/lilibot
3.5) cd ./lilibot/catkin_ws && catkin_make
3.6) Run simularion, 
    a) roscore
    b) cd vrep_simualtion && vrep lilibot-V5

4) How to implement your controller

 Just edit the file of lilibot_controller_node.cpp
you can find this file at src/lilibot_controller/src/lilibot_controller_node.cpp

