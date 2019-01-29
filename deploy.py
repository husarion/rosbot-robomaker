import boto3
import sys

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
        if fleet_name==default_fleet_name:
            rosbot_fleet_found = True
            rosbot_fleet_arn = fleet_arn
        
    if rosbot_fleet_found==False:
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
        if robot_name==rosbot_name:
            print('    Found ' + rosbot_name + ' with ID: ' + robot_arn)
            print('    ' + rosbot_name + ' belongs to fleet: ' + robot.get('fleetArn', 'noFleetDefined'))
            return robot_arn
    print(rosbot_name + ' not found, please create robot first')
    return None

def update_robot_fleet(robomaker, robot_arn, fleet_arn):
    describe_robot_response = robomaker.describe_robot(
        robot = robot_arn
        )
    robot_fleet_arn = describe_robot_response.get('fleetArn', 'undefined')
    if robot_fleet_arn=='undefined':
        print('Need to register robot in fleet')
        response = robomaker.register_robot(
            fleet=fleet_arn,
            robot=robot_arn
        )
    else:
        if robot_fleet_arn==fleet_arn:
            print('Robot belongs to: ' + robot_fleet_arn + ' fleet. Accepted!')
        else:
            print('Robot belongs to: ' + robot_fleet_arn + ' fleet. Will do new registration.')
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
        if app_name==robot_application.get('name'):
            app_arn = robot_application.get('arn')
            print('    Found app with ID: ' + app_arn)
            return app_arn
    return None
    
def get_current_application_version(robomaker, application_arn):
    create_robot_application_version_response = robomaker.create_robot_application_version(
        application=application_arn
    )
    application_version = create_robot_application_version_response.get('version')
    print('    Application version: ' + str(application_version))
    return application_version

def create_deployment(robomaker, fleet_arn, app_arn, app_ver, tutorial_number):
    package_name='tutorial_pkg'
    launch_file='tutorial_' + str(tutorial_number) + '_core.launch'
    master_uri_id='ROS_MASTER_URI'
    master_uri_value='http://master:11311'
    ipv6_id='ROS_IPV6'
    ipv6_value='on'
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

def add_armhf_to_app(robomaker, app_arn):
    response = robomaker.describe_robot_application(
        application=app_arn
    )
    print(response)
    return

def start_deployment():
    # Begin working with robomaker
    print("Init robomaker")
    robomaker_client = boto3.client('robomaker')
    
    tutorial_num = 6
    fleet_arn = get_fleet_arn(robomaker_client, 'ROSbotFleet')
    response = robomaker_client.describe_fleet(fleet=fleet_arn)
    print('Clean fleet')
    for robot in response.get('robots'):
        deregister_robot_response = robomaker_client.deregister_robot(
                fleet=fleet_arn,
                robot=robot.get('arn')
        )
    print("Fleet empty, now desired robot will be added")
    rosbot_arn = get_robot_arn(robomaker_client, 'ROSbot')
    if rosbot_arn:
        update_robot_fleet(robomaker_client, rosbot_arn, fleet_arn)
    else:
        print('No robot found')
        return None
    application_arn = get_robot_application_arn(robomaker_client, 'RoboMakerROSbotTutorialRobot')
    add_armhf_to_app(robomaker_client, application_arn)
    app_version = get_current_application_version(robomaker_client, application_arn)
    create_deployment(robomaker_client, fleet_arn, application_arn, app_version, tutorial_num)
    
if __name__ == "__main__":
    start_deployment()