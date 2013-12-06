#!/bin/bash

ENV_FILE_NAME="./setenv.sh"
rm ${ENV_FILE_NAME}

# List of catkin packages to build
# Order can be important
PACKAGES=("syllo" "blueview")

# Descend into each catkin package and build it
for i in "${PACKAGES[@]}"
do
    :
    pushd "./src/${i}/catkin_ws" >& /dev/null
    ./clean.sh
    popd >& /dev/null
done
