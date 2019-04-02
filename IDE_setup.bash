#!/bin/bash

if [ $# -eq 3 ]
then
    BUCKET_NAME="$1"
    IAM_ROLE="$2"
    DEPLOYMENT_ROLE_ARN="$3"
    echo "Bucket name is " $BUCKET_NAME
    echo "IAM role is " $IAM_ROLE
    echo "Deployment role ARN is " $DEPLOYMENT_ROLE_ARN
else
    echo "Please run this script with two arguments"
    echo "./IDE_setup.sh BUCKET_NAME IAM_ROLE"
    exit 1
fi

# install boto3
pip install -U boto3
pip3 install -U colcon-bundle

# update the submodules
cd ~/environment/RoboMakerROSbotProject
git submodule update --recursive --remote

aws greengrass associate-service-role-to-account --role-arn $DEPLOYMENT_ROLE_ARN

# configure project
cd ~/environment/RoboMakerROSbotProject/
python configure_project.py --bucket $BUCKET_NAME --iam $IAM_ROLE

while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    echo "wait for other apt instances to finish"
    sleep 1
done

# build X86_64 architecture
cd ~/environment/RoboMakerROSbotProject/robot_ws
. /opt/ros/kinetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle --apt-package-blacklist rosbot.blacklist

# copy X86_64 bundle to S3 bucket
cd ~/environment/RoboMakerROSbotProject
aws s3 cp robot_ws/bundle/output.tar.gz s3://$BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.tar.gz

# prepare docker for armhf compilation
if [[ $(docker ps -q) ]]
then
    EXISTING_CONTAINER_ID=$(docker ps -q)
    echo "Use currently running container: $EXISTING_CONTAINER_ID"
elif [[ $(docker ps -q -a) ]]
then
    EXISTING_CONTAINER_ID=$(docker ps -q -a -l)
    echo "Restart existing container: $EXISTING_CONTAINER_ID"
    docker start $EXISTING_CONTAINER_ID
else
    echo "Create new container"
    cd /opt/robomaker/cross-compilation-dockerfile/
    sudo bin/build_image.bash
    cd ~/environment/RoboMakerROSbotProject
    EXISTING_CONTAINER_ID=$(sudo docker run -v $(pwd):/ws -dt ros-cross-compile:armhf)
fi

# start build docker
cd ~/environment/RoboMakerROSbotProject
echo "Cross compile started with ID: " $EXISTING_CONTAINER_ID
docker exec $EXISTING_CONTAINER_ID ws/armhf.bash

# copy armhf bundle to S3 bucket
cd ~/environment/RoboMakerROSbotProject
aws s3 cp robot_ws/armhf_bundle/output.tar.gz s3://$BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.armhf.tar.gz

# start deployment job
cd ~/environment/RoboMakerROSbotProject
python deploy.py $BUCKET_NAME --robot ROSbotOffice
