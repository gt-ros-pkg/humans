#!/bin/bash

URDF=$(readlink -f `rospack find g500arm5_description`/urdf/g500arm5.urdf)
SDF=$(readlink -f `rospack find g500arm5_description`/urdf/g500arm5.sdf)

#cat $(gzsdf 

#roslaunch g500arm5_gazebo g500arm5.launch
