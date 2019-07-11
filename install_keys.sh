#!/bin/bash

chmod 400 .ssh/rosbot_key.pem

sudo cp /home/husarion/.ssh/rosbot_key.pem /home/ggc_user/.ssh/rosbot_key.pem
sudo chown ggc_user /home/ggc_user/.ssh/rosbot_key.pem
sudo chmod 400 /home/ggc_user/.ssh/rosbot_key.pem

sudo mkdir -p /home/ggc_user/.ssh
sudo touch /home/ggc_user/.ssh/known_hosts
sudo chown ggc_user /home/ggc_user/
sudo chown ggc_user /home/ggc_user/.ssh/
sudo chown ggc_user /home/ggc_user/.ssh/known_hosts

ssh-keyscan -H ec2-instance >> ~/.ssh/known_hosts
sudo -H -u ggc_user bash -c 'ssh-keyscan -H ec2-instance >> ~/.ssh/known_hosts'
