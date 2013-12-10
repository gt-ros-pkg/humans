#!/bin/bash

if [ $# -lt 1 ]
then
echo "usage: ./rqt-plugin-generator.sh <rqt-plugin-name>"
echo "       Don't include the \"rqt\" portion in your plugin name"
echo "example: ./rqt-plugin-generator.sh my_cool_plugin"
exit 
fi

cp -r ./template ./rqt_$@

pushd ./rqt_$@ > /dev/null

# Rename include files
mv ./include/rqt_template/template.h ./include/rqt_template/$@.h
mv ./include/rqt_template ./include/rqt_$@

# Rename src files
mv ./src/rqt_template/template.cpp ./src/rqt_template/$@.cpp
mv ./src/rqt_template/template.ui ./src/rqt_template/$@.ui
mv ./src/rqt_template ./src/rqt_$@

# Rename script file
mv ./scripts/rqt_template ./scripts/rqt_$@

find . -type f -exec sed -i "s/(>>>APP-NAME<<<)/$@/g" {} \;

popd > /dev/null

echo "Generated project: $@"
