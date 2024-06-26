#!/bin/bash
# Some of us are lazy lets do everything in one script

# Use from the root of the px4 repo
# Calling : ~/px4$ "./boards/modalai/voxl2/scripts/build_and_deploy.sh"
# Requires sudo if your underlying docker enviroment is not rootless

SCRIPT_LOCATION="boards/modalai/voxl2/scripts"
CONTAINER_NAME="voxl2_rb5"
IMAGE_NAME="rb5-flight-px4-build-docker"

# Run the docker container but dont enter docker
# ./$SCRIPT_LOCATION/run-docker.sh &

# Run this from the px4 project top level directory
docker run -i -d --name=$CONTAINER_NAME --rm -v `pwd`:/usr/local/workspace $IMAGE_NAME
echo "Docker Container $CONTAINER_NAME Started"

# Clean the build
echo "CLEAN"
docker exec $CONTAINER_NAME "./$SCRIPT_LOCATION/clean.sh"

# Build the dependecies
echo "BUILD VOXL2 Dependencies"
docker exec $CONTAINER_NAME "./$SCRIPT_LOCATION/build-deps.sh"

# Build the apps (ubuntu side)
echo "BUILD VOXL2 APPS Side (Ubuntu Side)"
docker exec $CONTAINER_NAME "./$SCRIPT_LOCATION/build-apps.sh"

# Build the slpi (dsp side)
echo "BUILD VOXL2 SLPI Side (DSP Side)"
docker exec $CONTAINER_NAME "./$SCRIPT_LOCATION/build-slpi.sh"

# STOP the docker container
docker stop $CONTAINER_NAME &

# this removes all created containers
# docker rm $(docker ps -a -q)

# Deploy the package to target
# FIXME ASK user here if they want to deploy to a voxl2
./$SCRIPT_LOCATION/install-voxl.sh

# Reboot the voxl2
adb shell reboot

# Archive and Rename the package for use later on


# Double check the docker container has been removed from the list
# If it hasn't been removed it wont start the next time with the know $CONTAINER_NAME
# Will give error if already removed

# docker rm $CONTAINER_NAME
