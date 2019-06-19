#!/bin/bash

if [ $# -eq 3 ]
then
    BUCKET_NAME="$1"
    IAM_ROLE="$2"
    DEPLOYMENT_ROLE_ARN="$3"
    echo "Bucket name is " $BUCKET_NAME
    echo "IAM role is " $IAM_ROLE
    echo "Deployment role ARN is " $DEPLOYMENT_ROLE_ARN
elif [ $# -eq 4 ]
then
    BUCKET_NAME="$1"
    IAM_ROLE="$2"
    DEPLOYMENT_ROLE_ARN="$3"
    LAUNCH_FILE_NAME="$4"
    echo "Bucket name is " $BUCKET_NAME
    echo "IAM role is " $IAM_ROLE
    echo "Deployment role ARN is " $DEPLOYMENT_ROLE_ARN
    echo "Launch file name is " $LAUNCH_FILE_NAME
else
    echo "Please run this script with three of four arguments"
    echo "./IDE_setup.sh BUCKET_NAME IAM_ROLE DEPLOYMENT_ROLE_ARN"
    echo "./IDE_setup.sh BUCKET_NAME IAM_ROLE DEPLOYMENT_ROLE_ARN LAUNCH_FILE_NAME"
    exit 1
fi

FIRST_RUN=0
if [ ! -d robot_ws/log ]
then
    # Script first run
    FIRST_RUN=1
    while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        echo "wait for other apt instances to finish"
        sleep 1
    done
    # Need to make configurations
    sudo apt update
    sudo apt upgrade -y
    sudo -H pip install -U boto3
    sudo -H pip3 install -U colcon-bundle
fi

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
aws s3 cp robot_ws/bundle/output.tar s3://$BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.tar

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

if [ $FIRST_RUN -eq 1 ]
then
    docker exec $EXISTING_CONTAINER_ID apt update
    docker exec $EXISTING_CONTAINER_ID apt upgrade -y
    docker exec $EXISTING_CONTAINER_ID pip3 install -U colcon-bundle
fi

# start build docker
cd ~/environment/RoboMakerROSbotProject
echo "Cross compile started with ID: " $EXISTING_CONTAINER_ID
docker exec $EXISTING_CONTAINER_ID ws/armhf.bash

# copy armhf bundle to S3 bucket
cd ~/environment/RoboMakerROSbotProject
aws s3 cp robot_ws/armhf_bundle/output.tar s3://$BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.armhf.tar

# start deployment job
cd ~/environment/RoboMakerROSbotProject

if [[ $LAUNCH_FILE_NAME ]]
then 
    python deploy.py $BUCKET_NAME --robot ROSbot --launch $LAUNCH_FILE_NAME
else
    python deploy.py $BUCKET_NAME --robot ROSbot
fi