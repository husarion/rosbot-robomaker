# RoboMaker ROSbot project

This project contains ROSbot model along with launch files required to launch tutorials on [AWS RoboMaker](https://aws.amazon.com/robomaker/).
Currently only the tutorial 6 is tested, other tutorials will be added soon.

## How to use it

You will need RoboMaker development environmet, if you do not have it yet create it according to [Create a development environment](https://docs.aws.amazon.com/robomaker/latest/dg/gs-build.html?shortFooter=true#gs-build-createide) manual.

You will also need S3 bucket and IAM role prepared for simulations.
In case you need help, you can check out our [tutorial](https://husarion.com/tutorials/other-tutorials/run-ros-tutorials-using-aws-robomaker/).

- Open the development environmet and go to terminal tab
- Clone this repository in `~/environment/` directory
```
cd ~/environment/
git clone --recurse-submodules https://github.com/lukaszmitka/RoboMakerROSbotProject.git
```

- Configure the project using provided script:

```
cd RoboMakerROSbotProject/
python configure_project.py 
```

The script will ask you for S3 bucket name and an IAM role identifier. 

Go to menu `RoboMaker Run` -> `Workflow` -> `ROSbotTutorial - Build and Bundle All` to build required packages.

Go to menu `RoboMaker Run` -> `Launch Simulaiton` -> `ROSbotTutorial6` to start simulation job.
