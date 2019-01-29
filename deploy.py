from collections import OrderedDict
import boto3
import sys
import argparse


def get_fleet_arn(robomaker, default_fleet_name):
    rosbot_fleet_found = False
    rosbot_fleet_arn = ''
    print('Script will check if fleet named ' + default_fleet_name + ' exists.')
    print('If found, it will be used for further development, otherwise it will be created')
    list_fleets_response = robomaker.list_fleets()
    print("Found existing fleets:")
    for fleet in list_fleets_response.get('fleetDetails'):
        fleet_name = fleet.get('name')
        fleet_arn = fleet.get('arn')
        print("    " + fleet_name + ' ID: ' + fleet_arn)
        if fleet_name == default_fleet_name:
            rosbot_fleet_found = True
            rosbot_fleet_arn = fleet_arn

    if rosbot_fleet_found == False:
        print(default_fleet_name + ' not found, will create new fleet')
        create_fleet_response = robomaker.create_fleet(
            name=default_fleet_name
        )
        rosbot_fleet_arn = create_fleet_response.get('arn')
        print("    Created fleet with ID: " + rosbot_fleet_arn)
    else:
        print(default_fleet_name + 'found with ID: ' + rosbot_fleet_arn)
    return rosbot_fleet_arn


def get_robot_arn(robomaker, rosbot_name):
    print('Search for robot: ' + rosbot_name)
    list_robots_response = robomaker.list_robots()
    for robot in list_robots_response.get('robots'):
        robot_name = robot.get('name')
        robot_arn = robot.get('arn')
        if robot_name == rosbot_name:
            print('    Found ' + rosbot_name + ' with ID: ' + robot_arn)
            print('    ' + rosbot_name + ' belongs to fleet: ' +
                  robot.get('fleetArn', 'noFleetDefined'))
            return robot_arn
    print(rosbot_name + ' not found, please create robot first')
    return None


def update_robot_fleet(robomaker, robot_arn, fleet_arn):
    describe_robot_response = robomaker.describe_robot(
        robot=robot_arn
    )
    robot_fleet_arn = describe_robot_response.get('fleetArn', 'undefined')
    if robot_fleet_arn == 'undefined':
        print('Need to register robot in fleet')
        response = robomaker.register_robot(
            fleet=fleet_arn,
            robot=robot_arn
        )
    else:
        if robot_fleet_arn == fleet_arn:
            print('Robot belongs to: ' + robot_fleet_arn + ' fleet. Accepted!')
        else:
            print('Robot belongs to: ' + robot_fleet_arn +
                  ' fleet. Will do new registration.')
            deregister_robot_response = robomaker.deregister_robot(
                fleet=robot_fleet_arn,
                robot=robot_arn
            )
            response = robomaker.register_robot(
                fleet=fleet_arn,
                robot=robot_arn
            )
    return


def get_robot_application_arn(robomaker, app_name):
    print('Search for robot application: ' + app_name)
    list_robot_applications_response = robomaker.list_robot_applications()
    for robot_application in list_robot_applications_response.get('robotApplicationSummaries'):
        if app_name == robot_application.get('name'):
            app_arn = robot_application.get('arn')
            print('    Found app with ID: ' + app_arn)
            return app_arn
    return None


def get_current_application_version(robomaker, application_arn):
    create_robot_application_version_response = robomaker.create_robot_application_version(
        application=application_arn
    )
    application_version = create_robot_application_version_response.get(
        'version')
    print('    Application version: ' + str(application_version))
    return application_version


def create_deployment(robomaker, fleet_arn, app_arn, app_ver, tutorial_number):
    package_name = 'tutorial_pkg'
    launch_file = 'tutorial_' + str(tutorial_number) + '_core.launch'
    master_uri_id = 'ROS_MASTER_URI'
    master_uri_value = 'http://master:11311'
    ipv6_id = 'ROS_IPV6'
    ipv6_value = 'on'
    response = robomaker.create_deployment_job(
        deploymentConfig={
            'concurrentDeploymentPercentage': 20,
            'failureThresholdPercentage': 25
        },
        fleet=fleet_arn,
        deploymentApplicationConfigs=[
            {
                'application': app_arn,
                'applicationVersion': app_ver,
                'launchConfig': {
                    'packageName': package_name,
                    'launchFile': launch_file,
                    'environmentVariables': {
                        master_uri_id: master_uri_value,
                        ipv6_id: ipv6_value
                    }
                }
            },
        ]
    )
    print(response)


def add_armhf_to_app(robomaker, app_arn, bucket_id, prefix):
    describe_robot_application_response = robomaker.describe_robot_application(
        application=app_arn
        )
    print('Will update robot application with')
    print('Name: ' + describe_robot_application_response.get('name'))
    sources_list = []
    for source in describe_robot_application_response.get('sources'):
        s = OrderedDict([
            ('s3Bucket', source.get('s3Bucket')),
            ('s3Key', source.get('s3Key')),
            ('architecture', source.get('architecture'))
        ])
        sources_list.append(s)
        print('Source:')
        print('    s3Bucket: ' + source.get('s3Bucket'))
        print('    s3Key: ' + source.get('s3Key'))
        print('    architecture: ' + source.get('architecture'))

    armhf_source = OrderedDict([
        ('s3Bucket', bucket_id),
        ('s3Key', prefix+'/output.armhf.tar.gz'),
        ('architecture', 'ARMHF')
    ])
    sources_list.append(armhf_source)

    print('Sources list contain ' + str(len(sources_list)) + ' objects')

    update_robot_application_response = robomaker.update_robot_application(
        application=app_arn,
        sources=sources_list,
        robotSoftwareSuite={
            'name': 'ROS',
            'version': 'Kinetic'
        }
    )
    return


def create_robot_app(robomaker, app_name, bucket, prefix):
    sources_list = []

    x86_64_source = OrderedDict([
        ('s3Bucket', bucket),
        ('s3Key', prefix+'/output.tar.gz'),
        ('architecture', 'X86_64')
    ])
    sources_list.append(x86_64_source)

    armhf_source = OrderedDict([
        ('s3Bucket', bucket),
        ('s3Key', prefix+'/output.armhf.tar.gz'),
        ('architecture', 'ARMHF')
    ])
    sources_list.append(armhf_source)
    create_robot_application_response = robomaker.create_robot_application(
        name=app_name,
        sources=sources_list,
        robotSoftwareSuite={
            'name': 'ROS',
            'version': 'Kinetic'
        }
    )
    return create_robot_application_response.get('arn')


def start_deployment(tutorial_number, user_fleet_name, user_robot_name, s3bucket):
    # Begin working with robomaker
    print("Init robomaker")
    robomaker_client = boto3.client('robomaker')
    fleet_arn = get_fleet_arn(robomaker_client, user_fleet_name)
    response = robomaker_client.describe_fleet(fleet=fleet_arn)
    app_key_prefix = 'RoboMakerROSbotProject/robot_ws/bundle'
    print('Clean fleet')
    for robot in response.get('robots'):
        deregister_robot_response = robomaker_client.deregister_robot(
            fleet=fleet_arn,
            robot=robot.get('arn')
        )
    print("Fleet empty, now desired robot will be added")
    rosbot_arn = get_robot_arn(robomaker_client, user_robot_name)
    if rosbot_arn:
        update_robot_fleet(robomaker_client, rosbot_arn, fleet_arn)
    else:
        print('No robot found')
        return None
    application_arn = get_robot_application_arn(
        robomaker_client, 'RoboMakerROSbotTutorialRobot')
    if application_arn:
        add_armhf_to_app(robomaker_client, application_arn,
                         s3bucket, app_key_prefix)
    else:
        application_arn = create_robot_app(
            robomaker_client, 'RoboMakerROSbotTutorialRobot', s3bucket, app_key_prefix)
    app_version = get_current_application_version(
        robomaker_client, application_arn)
    create_deployment(robomaker_client, fleet_arn,
                      application_arn, app_version, tutorial_number)


if __name__ == "__main__":
    tutorial_num = 8
    robot_name = ''
    fleet_name = ''
    bucket_name = ''
    parser = argparse.ArgumentParser()
    parser.add_argument("bucket", help="Bucket where app bundles are stored, required")
    parser.add_argument("--tutorial", help="Number of tutorial to be used for deployment, default is 8", type=int)
    parser.add_argument("--fleet", help="Fleet name to be used, default is ROSbotFleet")
    parser.add_argument("--robot", help="Robot name to be used, default is ROSbot")
    args = parser.parse_args()

    bucket_name = args.bucket

    if args.tutorial:
        tutorial_num = args.tutorial
    else:
        tutorial_num = 8

    if args.fleet:
        fleet_name = args.fleet
    else:
        fleet_name = 'ROSbotFleet'

    if args.robot:
        robot_name = args.robot
    else:
        robot_name = 'ROSbot'

    start_deployment(tutorial_num, fleet_name, robot_name, bucket_name)
