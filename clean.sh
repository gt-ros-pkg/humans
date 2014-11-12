#!/bin/bash

ENV_FILE_NAME="./setenv.sh"
rm ${ENV_FILE_NAME}

# List of catkin packages to build
# Order can be important
PACKAGES=("syllo" "input" "videoray" "syllo_rqt" "blueview" "sandbox" "syllo_uwsim" "underwater")

# Traverse through PACKAGES in reverse order due to dependencies
# and run clean
for (( idx=${#PACKAGES[@]}-1 ; idx>=0 ; idx-- )) ; do
    pushd "./src/${PACKAGES[idx]}/catkin_ws" >& /dev/null
    ./clean.sh
    popd >& /dev/null
done

# Remove .catkin_worspace files
find . -name ".catkin_workspace" | xargs rm
