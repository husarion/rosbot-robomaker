#!/usr/bin/env sh

if [ $# -eq 2 ]
then
    BUCKET_NAME="$1"
    IAM_ROLE="$2"
    echo "Bucket name is " $BUCKET_NAME
    echo "IAM role is " $IAM_ROLE
else
    echo "Please run this script with two arguments"
    echo "./IDE_setup.sh BUCKET_NAME IAM_ROLE"
fi

# install boto3 
pip install boto3

# update the submodules
cd ~/environment/RoboMakerROSbotProject
git submodule update --recursive --remote

# configure project
cd ~/environment/RoboMakerROSbotProject/
python configure_project.py --bucket $BUCKET_NAME --iam $IAM_ROLE

# prepare docker for armhf compilation
cd /opt/robomaker/cross-compilation-dockerfile/
sudo bin/build_image.bash

# stop and delete all running docker containers
docker stop $(docker ps -aq)
docker rm $(docker ps -aq)

# start build docker
cd ~/environment/RoboMakerROSbotProject
CONTAINER_ID=$(sudo docker run -v $(pwd):/ws -dt ros-cross-compile:armhf)
echo "Cross compile started with ID: " $CONTAINER_ID
docker exec $CONTAINER_ID  ws/armhf.sh

# copy armhf bundle to S3 bucket
cd ~/environment/RoboMakerROSbotProject
aws s3 cp robot_ws/armhf_bundle/output.tar.gz s3://$BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.armhf.tar.gz

