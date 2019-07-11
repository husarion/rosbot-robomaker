#!/bin/bash

EC2_PUBLIC_DNS=$(python run_instance.py | tail -1)
# EC2_PUBLIC_DNS=ec2-34-243-188-192.eu-west-1.compute.amazonaws.com
echo "Address:" $EC2_PUBLIC_DNS
sudo apt install sshpass > /dev/null 2>&1
curl https://install.husarnet.com/install.sh | sudo bash > /dev/null 2>&1
sudo systemctl restart husarnet > /dev/null 2>&1

echo "Remove old rosbot_key.pem"
sudo rm ~/.ssh/rosbot_key.pem
cp rosbot_key.pem ~/.ssh/rosbot_key.pem
chmod 400 ~/.ssh/rosbot_key.pem

echo "Wait until instance is started"
sleep 3m
echo "Run ssh-keyscan"
ssh-keyscan -H $EC2_PUBLIC_DNS >> ~/.ssh/known_hosts

echo "scp install_packages_for_ec2.sh"
scp -i "~/.ssh/rosbot_key.pem" install_packages_for_ec2.sh ubuntu@$EC2_PUBLIC_DNS:install_packages_for_ec2.sh

echo "Execute install packages"
ssh -i "~/.ssh/rosbot_key.pem" ubuntu@$EC2_PUBLIC_DNS ./install_packages_for_ec2.sh

echo "call husarnet websetup on EC2 instance"
ssh -i "~/.ssh/rosbot_key.pem" ubuntu@$EC2_PUBLIC_DNS sudo husarnet websetup
read -p "Use above link to add EC2 instance to Husarnet, please name it 'ec2-instance', then press enter to continue"

sudo husarnet websetup
read -p "Use above link to add RoboMaker IDE to Husarnet, please name it 'robomaker-ide', then press enter to continue"

echo "ssh-keyscan rosbot"
ssh-keyscan -H rosbot >> ~/.ssh/known_hosts
echo "rm .ssh/rosbot_key.pem @ rosbot"
sshpass -p husarion ssh husarion@rosbot sudo rm .ssh/rosbot_key.pem
echo "scp .ssh/rosbot_key.pem @ rosbot"
sshpass -p husarion scp rosbot_key.pem husarion@rosbot:.ssh/rosbot_key.pem
echo "scp install_keys.sh @ rosbot"
sshpass -p husarion scp install_keys.sh husarion@rosbot:install_keys.sh
echo "execute install_keys.sh @ rosbot"
sshpass -p husarion ssh husarion@rosbot ./install_keys.sh

echo "Configure EC2 done"