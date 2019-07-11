from collections import OrderedDict
import boto3
import sys
import argparse
import time

ec2_client = boto3.client('ec2')
response = ec2_client.describe_key_pairs(
    DryRun=False
)

for key_pair in response.get('KeyPairs'):
    key_name = key_pair.get("KeyName")
    if key_name == 'outsource_test':
        print("outsource_test key found")
    elif key_name == 'rosbot_key':
        print("rosbot_key key found, delete it now")
        key_pair_del_response = ec2_client.delete_key_pair(
            KeyName='rosbot_key',
            DryRun=False
        )
        del_key_metadata = key_pair_del_response.get('ResponseMetadata')
        del_key_status = del_key_metadata.get('HTTPStatusCode')
        if del_key_status == 200:
            print('rosbot_key deleted')
    else:
        print("KeyName not recognized")

print('Create new rosbot_key file')
create_key_pair_response = ec2_client.create_key_pair(
    KeyName='rosbot_key',
    DryRun=False
)
rosbot_key_material = create_key_pair_response.get('KeyMaterial')
if rosbot_key_material:
    rosbot_key_file  = open('rosbot_key.pem', 'w')
    rosbot_key_file.write(rosbot_key_material)
    rosbot_key_file.close() 
    print('rosbot_key saved to file rosbot_key.pem')

run_instance_response = ec2_client.run_instances(
    ImageId='ami-08660f1c6fb6b01e7',
    InstanceType='t2.micro',
    KeyName='rosbot_key',
    MaxCount=1,
    MinCount=1
)

instance_id = None
for instance in run_instance_response.get('Instances'):
    print("Instance DNS address:")
    print(instance.get('PublicDnsName'))
    print('Instance ID')
    instance_id = instance.get('InstanceId') 
    print(instance_id)

print ('Wait for instance to get DNS address')

instance_dns = None
while not instance_dns:
    describe_instances_response = ec2_client.describe_instances(
        InstanceIds=[
            instance_id
        ],
        DryRun=False
    )
    for reservation in describe_instances_response.get('Reservations'):
        for instance in reservation.get('Instances'):
            instance_dns = instance.get('PublicDnsName')
            time.sleep(1)
print(instance_dns)
