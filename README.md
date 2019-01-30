# RoboMaker ROSbot project

This project contains ROSbot model along with launch files required to launch tutorials on [AWS RoboMaker](https://aws.amazon.com/robomaker/) and deploy them to ROSbot with use of Greengrass.
Currently, tutorials 6, 7 and 8 are tested, other tutorials will be added soon.

Using Greengrass on ROSbot requires to follow some guides locates on varous places and making manual system modifiactions. We are working on making the process smooth and easy, stay tuned!

## Prerequisites

You will need RoboMaker development environmet, if you do not have it yet create it according to [Create a development environment](https://docs.aws.amazon.com/robomaker/latest/dg/gs-build.html?shortFooter=true#gs-build-createide) manual.

You will also need S3 bucket and IAM role prepared for simulations.
In case you need help, you can check out our [tutorial](https://husarion.com/tutorials/other-tutorials/run-ros-tutorials-using-aws-robomaker/).

By default, script will create fleet named 'ROSbotFleet' deploy app to robot named 'ROSbot'. Your environment need to have robot created, to create a robot follow [theese steps](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot.html?shortFooter=true#create-robot-steps).

## How to use it

- Open the development environmet and go to terminal tab
- Clone this repository in `~/environment/` directory:
```
cd ~/environment/
git clone --recurse-submodules https://github.com/lukaszmitka/RoboMakerROSbotProject.git
```

Start the configuration script:

```
cd ~/environment/RoboMakerROSbotProject/
./IDE_setup.bash <BUCKET_NAME> <IAM_ROLE>
```

The script will install all dependencies, configure project, build and set the deployment job.

## Components
You can also make your modifications to package.

- Start with updating all submodules:
```
git submodule update --recursive --remote
```

- Configure the project using `configure_project.py` script, this will setup build, bundle and simulation tasks:

```
cd RoboMakerROSbotProject/
python configure_project.py --bucket BUCKET_NAME --iam IAM_ROLE
```
Go to menu `RoboMaker Run` -> `Add or Edit Configurations`, choose `Switch config` button and select `roboMakerSettings.json` file from folder `/RoboMakerROSbotProject`. Choose `OK` and `Save`.

Go to menu `RoboMaker Run` -> `Workflow` -> `ROSbotTutorial - Build and Bundle All` to build required packages.

Go to menu `RoboMaker Run` -> `Launch Simulaiton` -> `ROSbotTutorial6` to start simulation job.

#### To deploy app onto ROSbot:

- Prepare docker build:
```
cd /opt/robomaker/cross-compilation-dockerfile/
sudo bin/build_image.bash
```

- Stop and delete all running docker containers
```
docker stop $(docker ps -aq)
docker rm $(docker ps -aq)
```

- Start docker container
```
cd ~/environment/RoboMakerROSbotProject
sudo docker run -v $(pwd):/ws -dt ros-cross-compile:armhf
```

- Build armhf application
```
cd ws/robot_ws
apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
exit
```

- Copy armhf bundle to S3 bucket
```
cd ~/environment/RoboMakerROSbotProject
aws s3 cp robot_ws/armhf_bundle/output.tar.gz s3://BUCKET_NAME/RoboMakerROSbotProject/robot_ws/bundle/output.armhf.tar.gz
```

- Start deployment job
```
cd ~/environment/RoboMakerROSbotProject
python deploy.py BUCKET_NAME
```