#!/bin/bash

# Create a directory named "abdaz"
mkdir abdaz
cd abdaz

# Add Mono repository and key
sudo apt-get update
sudo apt-get install apt-transport-https dirmngr gnupg ca-certificates -y
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt-get update

# Install Mono
sudo apt-get install mono-complete -y

# Install pip3
sudo apt-get install python3-pip -y

# Set Python 3 as default Python version
sudo apt-get install python-is-python3 -y

# Install pymavlink
pip install --upgrade pip wheel setuptools requests
pip3 install pymavlink
pip install mavproxy

# Install Mission Planner
wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip
unzip MissionPlanner-latest.zip
cd MissionPlanner
mono MissionPlanner.exe

# Install ArduPilot
cd ..
sudo apt-get install git -y
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
./Tools/scripts/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter

# Install FlightGear
sudo apt-get install flightgear -y

. ~/.profile
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
. ~/.bashrc

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
. ~/.bashrc

echo "Installation complete!"
