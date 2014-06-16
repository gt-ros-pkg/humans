#!/bin/bash

# Due to a bug with the gzsdf converter, which doesn't let us set the model's gravity to zero,
# we have to manually generate the sdf file and then remove the bad gravity tags.

XACRO=$(readlink -f `rospack find g500arm5_description`/urdf/g500arm5.xacro)
URDF=$(readlink -f `rospack find g500arm5_description`/urdf/g500arm5.urdf)
SDF=$(readlink -f `rospack find g500arm5_description`/urdf/g500arm5.sdf)

# Convert from xacro to URDF
rosrun xacro xacro.py ${XACRO} > ${URDF}

# Convert from URDF to SDF
gzsdf print ${URDF} > ${SDF}

# Delete all occurances of <gravity>1</gravity>
sed -i 's,<gravity>1</gravity>,,g' ${SDF}

roslaunch g500arm5_gazebo g500arm5.launch
