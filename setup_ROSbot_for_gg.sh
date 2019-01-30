#!/bin/bash
# It is required to meke below modifications in ROSbot system
# Reboot after executing script is necessary

if [ "$(id -u)" != 0 ]; then
    echo "Error: please run the installer as root."
    exit 1
fi

echo 'deb http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main
# deb-src http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main' > /etc/apt/sources.list.d/webupd8team-ubuntu-java-xenial.list

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EEA14886

apt update
curl -sL https://deb.nodesource.com/setup_6.x | bash -
apt install -y nodejs software-properties-common oracle-java8-installer python3-colcon-common-extensions python3-pip
pip3 install --upgrade colcon-common-extensions setuptools
ln /usr/bin/java /usr/bin/java8
ln /usr/bin/node /usr/bin/nodejs6.10

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' > /etc/udev/rules.d/rplidar.rules


echo 'KERNEL=="ttyUSB1", MODE:="0777", SYMLINK+="core2serial"
KERNEL=="ttyCORE2", MODE:="0777"' > /etc/udev/rules.d/core2serial.rules

adduser --system ggc_user
addgroup --system ggc_group

if ! grep -q 'fs.protected_hardlinks = 1' /etc/sysctl.d/98-rpi.conf; then
    echo 'fs.protected_hardlinks = 1' >> /etc/sysctl.d/98-rpi.conf
fi

if ! grep -q 'fs.protected_symlinks = 1' /etc/sysctl.d/98-rpi.conf; then
    echo 'fs.protected_symlinks = 1' >> /etc/sysctl.d/98-rpi.conf
fi

if ! grep -q 'cgroup_enable=memory cgroup_memory=1' /boot/cmdline.txt; then
    LINE=$(sed 's/$/ cgroup_enable=memory cgroup_memory=1/' /boot/cmdline.txt)
    echo $LINE > /boot/cmdline.txt
fi

wget https://d1onfpft10uf5o.cloudfront.net/greengrass-core/downloads/1.7.0/greengrass-linux-armv7l-1.7.0.tar.gz
tar -zxvf greengrass-linux-armv7l-1.7.0.tar.gz -C /

cd /greengrass/certs
wget -O root.ca.pem http://www.symantec.com/content/en/us/enterprise/verisign/roots/VeriSign-Class%203-Public-Primary-Certification-Authority-G5.pem