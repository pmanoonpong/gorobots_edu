#! /bin/sh

project_path=$(cd `dirname $0`; pwd)
file="$project_path/../controllers/catkin_ws/src/lilibot_controller/lilibot_controller_node"

if [ ! -f "$file" ];then
    ln "$project_path/../controllers/catkin_ws/devel/lib/lilibot_controller/lilibot_controller_node" $file
fi

