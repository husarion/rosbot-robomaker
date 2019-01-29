#!/usr/bin/python
from collections import OrderedDict
import json
import argparse


def create_robot_build_task():
    robot_ws_cfg = OrderedDict([
        ('workingDir', './RoboMakerROSbotProject/robot_ws'),
        ('cmdArgs', '')
    ])
    run_cfg_1 = OrderedDict([
        ('id', 'ROSbotTutorial_Cfg01'),
        ('name', 'ROSbotTutorial Robot'),
        ('type', 'colcon build'),
        ('cfg', robot_ws_cfg)
    ]
    )
    return run_cfg_1


def create_robot_bundle_task():
    robot_ws_cfg = OrderedDict([
        ('workingDir', './RoboMakerROSbotProject/robot_ws'),
        ('cmdArgs', '')
    ])
    run_cfg_2 = OrderedDict([
        ('id', 'ROSbotTutorial_Cfg02'),
        ('name', 'ROSbotTutorial Robot'),
        ('type', 'colcon bundle'),
        ('cfg', robot_ws_cfg)
    ]
    )
    return run_cfg_2


def create_simulation_build_task():
    simulation_ws_cfg = OrderedDict([
        ('workingDir', './RoboMakerROSbotProject/simulation_ws'),
        ('cmdArgs', '')
    ])
    run_cfg_3 = OrderedDict([
        ('id', 'ROSbotTutorial_Cfg03'),
        ('name', 'ROSbotTutorial Simulation'),
        ('type', 'colcon build'),
        ('cfg', simulation_ws_cfg)
    ]
    )
    return run_cfg_3


def create_simulation_bundle_task():
    simulation_ws_cfg = OrderedDict([
        ('workingDir', './RoboMakerROSbotProject/simulation_ws'),
        ('cmdArgs', '')
    ])
    run_cfg_4 = OrderedDict([
        ('id', 'ROSbotTutorial_Cfg04'),
        ('name', 'ROSbotTutorial Simulation'),
        ('type', 'colcon bundle'),
        ('cfg', simulation_ws_cfg)
    ]
    )
    return run_cfg_4


def get_simulation_object(bucket_name, iam_role):
    simulation = OrderedDict([
        ('outputLocation', bucket_name),
        ('failureBehavior', 'Fail'),
        ('maxJobDurationInSeconds', 900),
        ('iamRole', iam_role)
    ])
    return simulation


def getRobotSoftwareSuite():
    robotSoftwareSuite = OrderedDict([
        ('version', 'Kinetic'),
        ('name', 'ROS')
    ])
    return robotSoftwareSuite


def getSimulationSoftwareSuite():
    simulationSoftwareSuite = OrderedDict([
        ('name', 'Gazebo'),
        ('version', '7')
    ])
    return simulationSoftwareSuite


def getRenderingEngine():
    renderingEngine = OrderedDict([
        ('name', 'OGRE'),
        ('version', '1.x')
    ])
    return renderingEngine


def getSimLaunchConfig(tutorial_number):
    launch_file = 'tutorial_' + str(tutorial_number) + '_gazebo_world.launch'
    simLaunchConfig = OrderedDict([
        ('packageName', 'tutorial_pkg'),
        ('launchFile', launch_file)
    ])
    return simLaunchConfig


def getSimulationApp(bucket_name, tutorial_number):
    simulationApp = OrderedDict([
        ('name', 'RoboMakerROSbotTutorialSimulation'),
        ('s3Bucket', bucket_name),
        ('sourceBundleFile', './RoboMakerROSbotProject/simulation_ws/bundle/output.tar.gz'),
        ('architecture', 'X86_64'),
        ('launchConfig', getSimLaunchConfig(tutorial_number)),
        ('robotSoftwareSuite', getRobotSoftwareSuite()),
        ('simulationSoftwareSuite', getSimulationSoftwareSuite()),
        ('renderingEngine', getRenderingEngine())
    ])
    return simulationApp


def getRobotLaunchConfig(tutorial_number):
    launch_file = 'tutorial_' + str(tutorial_number) + '_gazebo_rosbot.launch'
    robotLaunchConfig = OrderedDict([
        ('packageName', 'tutorial_pkg'),
        ('launchFile', launch_file)
    ])
    return robotLaunchConfig


def getRobotApp(bucket_name, tutorial_number):
    robotApp = OrderedDict([
        ('name', 'RoboMakerROSbotTutorialRobot'),
        ('s3Bucket', bucket_name),
        ('sourceBundleFile', './RoboMakerROSbotProject/robot_ws/bundle/output.tar.gz'),
        ('architecture', 'X86_64'),
        ('robotSoftwareSuite', getRobotSoftwareSuite()),
        ('launchConfig', getRobotLaunchConfig(tutorial_number))
    ])
    return robotApp


def getSimCfg(bucket_name, tutorial_number, iam_role):
    cfg = OrderedDict([
        ('robotApp', getRobotApp(bucket_name, tutorial_number)),
        ('simulationApp', getSimulationApp(bucket_name, tutorial_number)),
        ('simulation', get_simulation_object(bucket_name, iam_role))
    ])
    return cfg


def getSimulationJob(bucket_name, tutorial_number, iam_role):
    simID = 'ROSbotTutorial_SimulationJob_' + str(tutorial_number)
    simName = 'ROSbotTutorial' + str(tutorial_number)
    simulationJob = OrderedDict([
        ('id', simID),
        ('name', simName),
        ('type', 'simulation'),
        ('cfg', getSimCfg(bucket_name, tutorial_number, iam_role))
    ])
    return simulationJob


def main():
    user_bucket_name = ''
    user_iam_role = ''
    parser = argparse.ArgumentParser()
    parser.add_argument("--bucket", help="Identifier for bucket containing app bundles")
    parser.add_argument("--iam", help="IAM role for RoboMaker")
    args = parser.parse_args()
    if args.bucket:
        user_bucket_name = args.bucket
    else:
        print "Enter name of bucket containing app bundles:"
        user_bucket_name = raw_input()
    if args.iam:
        user_iam_role = args.iam
    else:
        print "Enter IAM role (string like 'arn:aws:iam::xxxxxxxxxxxx:role/xxxxxxxxxxxxx)' :"
        user_iam_role = raw_input()

    run_config_1 = create_robot_build_task()
    run_config_2 = create_robot_bundle_task()
    run_config_3 = create_simulation_build_task()
    run_config_4 = create_simulation_bundle_task()


    workflow_1 = OrderedDict([
        ('id', 'ROSbotTutorial_wf1'),
        ('type', 'workflow'),
        ('name', 'ROSbotTutorial - Build and Bundle All'),
        ('runCfgIds',
            [
                run_config_1.get('id'),
                run_config_2.get('id'),
                run_config_3.get('id'),
                run_config_4.get('id')
            ])
    ])

    data = OrderedDict([
        ('runConfigurations',
            [
                run_config_1,
                run_config_2,
                run_config_3,
                run_config_4,
                getSimulationJob(user_bucket_name, 6, user_iam_role),
                getSimulationJob(user_bucket_name, 7, user_iam_role),
                getSimulationJob(user_bucket_name, 8, user_iam_role),
                workflow_1
            ])
    ])

    with open('roboMakerSettings.json', 'w') as outfile:
        json.dump(data, outfile, indent=2)


if __name__ == "__main__":
    main()
