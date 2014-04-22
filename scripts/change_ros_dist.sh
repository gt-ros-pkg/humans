#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Invalid arguments"
    echo "Example: ./change_ros_dist.sh groovy hydro"
    exit
fi

dist_from=$1
dist_to=$2

pushd ../ >& /dev/null

arr=( $(find -L ./ -samefile /opt/ros/${dist_from}/share/catkin/cmake/toplevel.cmake) )

for i in ${arr[@]}
do
    :
    rm $i
    ln -s /opt/ros/${dist_to}/share/catkin/cmake/toplevel.cmake $i
    echo $i
done

popd >& /dev/null
