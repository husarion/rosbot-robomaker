# RoboMaker ROSbot project

This project contains ROSbot model along with launch files required to launch tutorials on [AWS RoboMaker](https://aws.amazon.com/robomaker/) and deploy them to ROSbot with use of Greengrass.
Currently, tutorials 6, 7 and 8 are tested, other tutorials will be added soon.

Using Greengrass on ROSbot requires to follow some guides locates on varous places and making manual system modifiactions. We are working on making the process smooth and easy, stay tuned!

#### Creating S3 Bucket

You will need S3 bucket to store your application bundles. To create S3 bucket:
- Sign in to [AWS S3 console](https://console.aws.amazon.com/s3/)
- Choose **Create bucket**
- In field **Bucket name** type DNS style name like `yourusername-bucket-robomaker` (it must be unique accross all names in Amazon S3, do not use `_` and `.` in the name)- From **Region dropdown** menu choose entry appropriate to your localization.
- Proceed through creator, do not modify default values.
- When you create bucket, open it, by clicking its name, it should be empty now. 
- Note the bucket name, you will need it later.
- Close the S3 console

#### Creating an IAM role for RoboMaker instance

You will need an IAM role to allow RoboMaker interact with other AWS services. To create IAM role:
- sign in to [AWS IAM console](https://console.aws.amazon.com/iam/)
- On the left panel choose **Roles**
- Click **Create role**
- From **Select type of trusted entity** menu select **AWS Service**
- From **Choose the service that will use this role** menu choose **EC2** 
- Proceed with **Next: Permissions** button
- In **Attach permissions policies** dialog, add (You can use filter to search for them):
    - CloudWatchFullAccess
    - AWSRoboMakerFullAccess
    - AmazonS3FullAccess

- Proceed with **Next: Tags** and **Next: Review**.
    - In field **Role name** type `robomaker_role`.
    - In **Role description** type `Allows RoboMaker instances to call AWS services on your behalf.`.
    - Make sure that in **Policies** section you have three entries.
- Proceed with **Create role**, you will be redirected to **Roles** view.
- Open role setting by clicking its name and choose tab **Trust relationships**. 
- Click button **Edit trust relationships** and edit policy document, find entry `ec2.amazonaws.com` and change it to `robomaker.amazonaws.com`.
- Click **Update trust policy** button. 
- Note the `Role ARN` entry, this will be required later.

#### Creating an IAM role for application deployment

Greengrass will need an IAM role to interact with other AWS S3 service during deployment. To create the role:
- sign in to [AWS IAM console](https://console.aws.amazon.com/iam/)
- On the left, choose **Policies**, then choose **Create policy**. Choose **JSON** tab and paste the code below. You will need to replace the `arn:aws:s3:::yourusername-bucket-robomaker` string with **ARN** of bucket that you created in first step:
```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Action": [
                "robomaker:UpdateRobotDeployment"
            ],
            "Resource": "*"
        },
        {
            "Effect": "Allow",
            "Action": [
                "s3:List*",
                "s3:Get*"
            ],
            "Resource": ["arn:aws:s3:::yourusername-bucket-robomaker/*"]
        }
    ]
}
``` 
- Proceed with **Review policy**.
- In **Name** field type 'ROSbot-deployment-policy'
- In **Description** field type 'Allow GreenGrass to deploy ROboMaker apps to ROSbot.'
- Choose **Create policy**. 
- Choose **Roles** and then choose **Create role**.
- In **Choose the service that will use this role** list find **Greengrass** and then choose **Next: Permissions**.
- In the **Permissions** page, select the policy `ROSbot-deployment-policy`. You can use filter to find it.
- Proceed with **Next: Tags** and **Next: Review**.
- In the **Review** page, type `ROSbot-deployment-role` in a **Role name** field.
- Proceed with **Create role**.
- Select the created role by clicking its name, then select the **Trust relationships** tab.
- Select **Edit trust relationship**.
- In **Policy document** field update with:
```
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Principal": {
        "Service": [
          "lambda.amazonaws.com",
          "greengrass.amazonaws.com"
        ]
      },
      "Action": "sts:AssumeRole"
    }
  ]
}
```
- Select **Update Trust Policy**.
- Close the IAM console

#### ROSbot setup in RoboMaker

ROSbot need some system modifications before Greengrass will be able to run and deploy applications. To configure ROSbot:
- sign in to the AWS RoboMaker [console]
- In the left navigation pane, choose **Fleet Management** and then choose **Robots**.
- Choose **Create robot*.
- In the **Name** field, type `ROSbot`.
- From the **Architecture** dropdown menu choose **ARMHF**.
- From the **AWS Greengrass group** dropdown menu choose **Create new**.
- In the **AWS Greengrass prefix** field type `ROSbot`
- In the **IAM role** select **ROSbot-deployment-role**
- Proceed with **Create**, you will be redirected to **Download your Core device** page.
- Choose **Download** button next to **Download and store your Core's security resources**
- You will get `ROSbot-setup.zip` file
- Copy the file to your ROSbot:
```
scp ROSbot-setup.zip husarion@ROSBOT_IP:ROSbot-setup.zip
```

#### ROSbot setup on device 

You will need to make some system configurations on device, connect to your ROSbot through `ssh` or remote desktop and open terminal:

- Unzip ROSbot security resources:
```
sudo unzip RobotName-setup.zip -d /greengrass
```
- Copy the `setup_ROSbot_for_gg.sh` file to your ROSbot and run it as root:
```
wget https://raw.githubusercontent.com/lukaszmitka/RoboMakerROSbotProject/master/setup_ROSbot_for_gg.sh
chmod a+x setup_ROSbot_for_gg.sh
sudo ./setup_ROSbot_for_gg.sh
```
- Start the GreenGrass:
```
sudo /greengrass/ggc/core/greengrassd start
```
- Leave the ROSbot turned on, it will wait for deployment.

#### Creating a RoboMaker IDE

Application will be built using the RoboMaker environment. To create the IDE:
- sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/home).
- On the left, expand **Development**, choose **Development environments**, and then choose **Create environment**.
- In the Create AWS RoboMaker development environment page, enter `rosbot_env` as the environment name.
- Accept the default Instance type (`m4.large`). You can select different instances type to improve bundling performance.
- In **VPC** dropdown list choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.
- Choose **Create** to create the AWS Cloud9 development environment.

#### Deploying the application

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`
- Open the development environmet with **Open environment** button.
- In the IDE, go to bash tab and clone this repository in `~/environment/` directory:
```
cd ~/environment/
git clone --recurse-submodules https://github.com/lukaszmitka/RoboMakerROSbotProject.git
```

- Start the configuration script. You need to provide bucket name and ARN of IAM role that you created for RoboMaker instance:

```
cd ~/environment/RoboMakerROSbotProject/
./IDE_setup.bash <BUCKET_NAME> <IAM_ROLE_FOR_ROBOMAKER>
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