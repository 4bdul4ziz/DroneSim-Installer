# Instructions to install ArduPilot, FlightGear, MAVProxy, Mission Planner, and pymavlink on Ubuntu

This repository contains a Bash script that installs the required components to simulate drone experiments done at VIT Chennai. Packages include: ArduPilot, FlightGear, MAVProxy, Mission Planner, and pymavlink on Ubuntu. The script has been tested on Ubuntu 20.04 LTS.

## Files

- `install.sh`: the Bash script that installs all the components

## Usage

1. Download the `install.sh` file from this repository to your Ubuntu machine.
2. Open a terminal and navigate to the directory where you downloaded the `install.sh` file.
3. Make the file executable by running the following command: `chmod +x install.sh`.
4. Execute the script with the following command: `./install.sh`.

The script will create a directory named `abdaz` and install all the components inside it. It will install Mono, pip3, python-is-python3, pymavlink, MAVProxy, Mission Planner, ArduPilot, and FlightGear.

## Disclaimer

This script has been tested on Ubuntu 20.04 LTS, but it may not work on other versions or distributions of Ubuntu. Use it at your own risk.

## Author
Abdul A.