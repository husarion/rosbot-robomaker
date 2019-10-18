# ROSbot - introduction #

ROSbot 2.0 is autonomous, open source robot platform. It can be used as a learning platform for Robot Operating System as well as a base for a variety of robotic applications such as research robots, inspection robots, custom service robots etc.

## Unboxing ##

What's in the box:

- carrying case
- ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
- Wi-Fi 2.4GHz antenna
- 3x 18650 Li-Ion rechargeable batteries
- universal charger with power adapter
- charging cable
- microSD card with the software for ROSbot
- USB to Ethernet adapter

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_unboxing.jpg"/></center></div>

## Rear panel description

In the picture below you can see names of the elements from the rear panel of the ROSbot.

![image](/images/ROSbot2_rear_panel.png)

## Hardware setup

### 1. Mounting the batteries

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna.

To mount the batteries turn ROSbot upside down and follow these steps:

1. Unscrew battery cover mounted with two screws.
2. Remove the battery cover.
3. Place batteries **accordingly to the polarization symbols (do it carefully!)**, keeping the black strip under the batteries.
4. Place battery cover and mount it with screws.

![image](/images/rosbot_battery.png)

### 2. Batteries charging guide

1. Connect the power adapter to the charger and the output cable between charger and ROSbot (2 connectors on charger side, 1 black connector to ROSbot charging port).

![image](/images/charger_1.png)
![image](/images/charger_2.png)

After this step is complete your charger should look like this:

![image](/images/rosbot_charger.jpg)

2. Use the first two buttons to select “LiPo BATT” mode and press [Start] button.
3. Use arrows to select “LiPo CHARGE” mode.
4. Press [Start] - the current value should start blinking. Use arrows to set the current to 1.5A.
5. Press [Start] again - the voltage value should start blinking. Select “11.1V(3S)” using arrows.
6. Press and hold [Start] for 2 seconds. The charger should now ask for confirmation. Press [Start] again. The charging process should begin now.
7. When charging is finished (after about 3 hours), the charger will generate a loud “beep” sound and will finish charging at the same time.

The picture below is a visualization of the mentioned steps.

![image](/images/charging.png)

### 3. Attaching the antenna

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

![image](/images/rosbot_antenna.png)

## Connecting to Wi-Fi network 

ROSbot is basically a computer running Ubuntu, so let's configure it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Use networking menu located on top-right of the screen to connect to a Wi-Fi network.
4. When connection is active, use networking menu again and choose **Connection Information** to find device IP address.
5. Note the ROSbot IP address, you will need it later.

## Flashing low level firmware

1. Disable `husarnet-configurator` and `husarion-shield services` and reboot your  ROSbot. These services are responsible for connection to the Husarion Cloud and they also control GPIO pins that are used for uploading the firmware. We will need direct  access to them. Run:

    ```bash
    sudo systemctl disable husarnet-configurator
    sudo systemctl stop husarnet-configurator
    sudo systemctl disable husarion-shield
    sudo reboot
    ```

2. Install necessary support libraries on your robot. In the terminal run:

    **ROSbot 2.0:**

    ```bash
    cd ~/ && git clone https://github.com/TinkerBoard/gpio_lib_python.git
    cd ~/gpio_lib_python && sudo python setup.py install --record files.txt
    ```

    **ROSbot 2.0 PRO:**

    ```bash
    cd ~/ && git clone https://github.com/vsergeev/python-periphery.git
    cd ~/python-periphery && sudo python setup.py install --record files.txt
    ```

    Restart the terminal after the installation.

3. Install `stm32loader` on your robot:

    ```bash
    cd ~/ && git clone https://github.com/byq77/stm32loader.git
    cd ~/stm32loader && sudo python setup.py install --record files.txt
    ```

    You can check if tool works by running following commands:

    **ROSbot 2.0:**

    ```bash
    sudo stm32loader -c tinker -f F4
    ```

    **ROSbot 2.0 PRO:**

    ```bash
    sudo stm32loader -c upboard -f F4
    ```

4. Download the firmware

Download the appropriate firmware to your ROSbot and save it in `/home/husarion/`:

- [`ROSbot 2.0`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.7.2.bin)
- [`ROSbot 2.0 Pro`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.7.2.bin)

5.  Flash the firmware

    To upload the firmware run:

    **ROSbot 2.0**:

    ```bash
    $ sudo stm32loader -c tinker -u -W
    ```

    ```bash
    $ sudo stm32loader -c tinker -e -w -v rosbot-2.0-***.bin
    ```

    **ROSbot 2.0 PRO**:

    ```bash
    sudo stm32loader -c upboard -u -W
    ```

    ```bash
    sudo stm32loader -c upboard -e -w -v rosbot-2.0-***.bin
    ```

    Wait until firmware is uploaded.


6. Unplug display, mouse and keyboard.

## RoboMaker ROSbot project ##

We have prepared the repository containing setup files along with ROSbot model and launch files required to use [Husarion ROS tutorials](https://husarion.com/tutorials/) on [AWS RoboMaker](https://aws.amazon.com/robomaker/) and deploy them to ROSbot with use of Greengrass.
Currently, tutorials [6 - SLAM navigation](https://husarion.com/tutorials/ros-tutorials/6-slam-navigation/), [7 - Path planning](https://husarion.com/tutorials/ros-tutorials/7-path-planning/) and [8 - Unknown environment exploration](https://husarion.com/tutorials/ros-tutorials/8-unknown-environment-exploration/) are tested, other tutorials will be added soon.


## Configure AWS Environment ##
Before we use AWS RoboMaker to build and deploy the tutorial applications, we must first set up the AWS environment. To simplify the configuration, we will use AWS CloudFormation. CloudFormation enables us to use a template file to define the configuration of our environment. We will use CloudFormation to create a bucket in Amaazon S3, as well as to create the necessary permissions in AWS Identity and Access Manager (IAM) that AWS RoboMaker requires to simulate and deploy our robot applications.

To deploy the template, sign in to the [CloudFormation console](https://console.aws.amazon.com/cloudformation/). Following the following steps to deploy the template:

1.  Download the template file from [here](https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/rosbot_tutorial_template.yaml).
2.  Click the **Create Stack** button.
3.  Under _Choose a template_, choose _Upload a template to Amazon S3_ and click **Choose File**.
4.  Browse to the rosbot_tutorial_template.yaml file you download in Step 1 above.
5.  Click **Next**.
6.  On the next screen, provide a _Stack name_. This should be something descriptive such as "ROSbot-setup".
7.  In the _S3BucketName_ field, provide a globally-unique name for the S3 bucket that will be created. This S3 bucket will be used to store your robot application bundles, as well as any logs that your robot may generate during simulation. Use a name unique to you, such as "&lt;user_id&gt;-rosbot-tutorial". Replace "&lt;user-id&gt;" with a unique string.
8.  Choose **Next**.
9.  On the Options page, leave all defaults and choose **Next**.
10. On the Review page, click the checkbox to acknowledge that CloudFormation will create IAM resources on your behalf.
11. Click **Create**.

After a few brief minutes, the stack will be created. When the status has changed to CREATE_COMPLETE, choose the stack you just created, and view its Outputs. You will see 3 key/value pairs. You will use these values later in this guide.

## ROSbot setup in RoboMaker

ROSbot need some system modifications before Greengrass will be able to run and deploy applications. To configure ROSbot:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Robots**.

![RoboMaker robots](/images/aws_tutorial_robomaker_1.png)

- Choose \*_Create robot_.
- In the **Name** field, type `ROSbot`.
- From the **Architecture** dropdown menu choose **ARMHF**.
- From the **AWS Greengrass group** dropdown menu choose **Create new**.
- In the **AWS Greengrass prefix** field type `ROSbot`.
- In the **IAM role** select **ROSbot-deployment-role**.

![RoboMaker create robot](/images/aws_tutorial_robomaker_2.png)

- Proceed with **Create**, you will be redirected to **Download your Core device** page.

![RoboMaker robot created](/images/aws_tutorial_robomaker_3.png)

- Choose **Download** button next to **Download and store your Core's security resources**, you will get `ROSbot-setup.zip` file.

- From **Download the current AWS Greengrass Core software** choose file for architecture **ARMv71**, you will get `greengrass-linux-armv7l-1.9.2.tar.gz` file.

- Both files need to be uploaded to ROSbot. The upload process will vary, depending on your host operating system.

### On Linux

Navigate to directory where the file is downloaded, by default it should be `~/Downloads`.

```
cd ~/Downloads
```

- Copy both files to your ROSbot, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
scp ROSbot-setup.zip husarion@ROSBOT_IP:ROSbot-setup.zip
scp greengrass-linux-armv7l-1.9.2.tar.gz husarion@ROSBOT_IP:greengrass-linux-armv7l-1.9.2.tar.gz
```


### On Windows

You will need an SCP client, download and install [WinSCP](https://winscp.net/eng/download.php).

Start WinSCP, you will see the login dialog:

![WinSCP login](/images/winscp1.png)

- From `File protocol` dropdown menu choose: `SFTP`.
- In `Host name` field provide rosbot IP address that you noted earlier, it is the value which we described as `ROSBOT_IP`.
- In `Port number` field provide `22`.
- In `User name` field provide `husarion`.
- In `Password` field provide `husarion`.

When all fields are filled up, click `Login` button to connect, you will see file manager view.

![WinSCP file manager](/images/winscp2.png)

In the left tab navigate to directory where you downloaded the ROSbot-setup.zip file. In the right tab navigate to `/home/husarion` directory.

Drag and drop the `ROSbot-setup.zip` and `greengrass-linux-armv7l-1.9.2.tar.gz` to the right tab.

When the transfer is finished, close the window.

## ROSbot setup on device

You will need to make some system configurations on device. Depending on your host system, you can connect to your ROSbot with different methods:

### On Linux

Open terminal and start `ssh` connection, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
ssh husarion@ROSBOT_IP
```

Proceed to **Device setup** section.

### On Windows

Press `WinKey` + `r` then type `mstsc`.

You will see a window appear:

![Windows RDP](/images/win_rdp.png)

Type in your device IP address and click connect.

You will see the ROSbot desktop, from the top menu, choose the `Applications` -> `Terminal`.

## Device setup

Type the following lines in the terminal to update the package list and upgrade packages:

```
sudo apt update
sudo apt dist-upgrade
```

In the terminal execute bellow commands:

- Copy the `setup_ROSbot_for_gg.sh` file to your ROSbot and run it as root:

```
wget https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/setup_ROSbot_for_gg.sh
chmod a+x setup_ROSbot_for_gg.sh
sudo ./setup_ROSbot_for_gg.sh
```

- Unzip ROSbot security resources:

```
cd ~
sudo unzip ROSbot-setup.zip -d /greengrass
```

- Restart ROSbot to apply chnages.

At this moment, ROSbot is ready to start AWS GreenGrass and accept incoming deployments.

Connect with ROSbot again, open terminal and start the GreenGrass:

```
sudo /greengrass/ggc/core/greengrassd start
```

Leave the ROSbot turned on, it will wait for deployment.

## Creating a RoboMaker IDE

Application will be built using the RoboMaker environment. To create the IDE:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/home)

![RoboMaker new IDE](/images/aws_tutorial_robomaker_4.png)

- On the left, expand **Development**, choose **Development environments**, and then choose **Create environment**.
- In the Create AWS RoboMaker development environment page, enter `rosbot_env` as the environment name.
- Accept the default Instance type (`m4.large`). You can select different instances type to improve bundling performance.
- In **VPC** dropdown list choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.

![RoboMaker create IDE](/images/aws_tutorial_robomaker_5.png)

- Choose **Create** to create the AWS Cloud9 development environment.

![RoboMaker IDE ready](/images/aws_tutorial_robomaker_6.png)

## Deploying the application

To deploy application, you will use RoboMaker environment created in previous step:

- Go to AWS RoboMaker home [console](https://console.aws.amazon.com/robomaker/home).

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`.

- Open the development environment with **Open environment** button.

![RoboMaker open IDE](/images/aws_tutorial_robomaker_8.png)

- In the IDE, go to bash tab and clone the `rosbot-robomaker` repository in `~/environment/` directory:

```
cd ~/environment/
git clone --recurse-submodules https://github.com/husarion/rosbot-robomaker.git RoboMakerROSbotProject
```

![RoboMaker open IDE](/images/aws_tutorial_robomaker_10.png)

- Start the configuration script. You need to provide the S3 bucket name and the ARNs of the IAM roles that were created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/RoboMakerROSbotProject/
./IDE_setup.bash <S3BucketName> <RoboMakerRole> <ROSbotDeploymentRole>
```

![RoboMaker open IDE](/images/aws_tutorial_robomaker_11.png)

The script will install all dependencies, configure project, build and set the deployment job.

When the script is done with its job, you can observe the deployment process:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Deployments**.
- When new deployment will appear, open it by clicking its name.
- Wait until deployment status changes to **Succeed** - ROSbot will start to explore environment.

![RoboMaker open IDE](/images/aws_tutorial_robomaker_12.png)

## Viewing the results

ROSbots mission, after completing this tutorial, is to autonomously explore the environment around it. It will drive to any location that is accessible and which cannot be observed from the already visited locations. It will create a map of that environment in the process using the data from the A2 LIDAR laser scanner mounted on top of it.

You may observe in real time how ROSbot is building the map. If you’d like to do that connect to ROSbot through a remote desktop client (this could be a `Remote Desktop Connection` on Windows or `Remmina` on Ubuntu).

Open terminal witm menu `Applications` -> `Terminal emulation` and execute:

```
rviz -d $(rospack find tutorial_pkg)/rviz/tutorial_8.rviz
```

You will see the Rviz visualization tool with created map and a planned trajectory.

You can see an example map being created on the screenshot below.

![RoboMaker exploration screenshot](/images/aws_tutorial_robomaker_13.png)

Map is considered complete, when there is no traversable gaps within the map boundaries.

When the map is complete, ROSbot will stop and wait for further commands, it is possible to set a destination point on a map using **`2D Nav Goal`** button in **Rviz**.

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
