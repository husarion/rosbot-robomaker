## Components

If you want to modify script or run only some of its elements, below guide may be helpful for you.

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