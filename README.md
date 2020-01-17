# Magang_Programming
Materi magang programming Bayucaraka ITS 2020
Referensi[
- [Dronekit](https://dronekit-python.readthedocs.io/en/latest/)
- Droneki-Python [github-repo](https://github.com/dronekit/dronekit-python)

# Python Dronekit
## About Dronekit and Compatibility
DroneKit-Python is an open source and community-driven project. DroneKit-Python allows developers to create apps that run on an onboard companion computer and communicate with the ArduPilot flight controller using a low-latency link. DroneKit-Python is compatible with vehicles that communicate using the MAVLink protocol (including most vehicles made by 3DR.

The API communicates with vehicles over MAVLink. It provides programmatic access to a connected vehicle’s telemetry, state and parameter information, and enables both mission management and direct control over vehicle movement and operations.

## API Features
- Connect to a vehicle (or multiple vehicles) from a script
- Get and set vehicle state/telemetry and parameter information.
- Receive asynchronous notification of state changes.
- Guide a UAV to specified position (GUIDED mode).
- Send arbitrary custom messages to control UAV movement and other hardware (GUIDED mode).
- Create and manage waypoint missions (AUTO mode).
- Override RC channel settings

# Getting Started
## Installing Python
## Installig Dronekit-Python
### For Windows
- Make sure you have installed Python 3 (Cause we are using Python 3)
- Install python3-pip
```
python get-pip.py
```
- Install Dronekit-Python
```
pip install dronekit
```
### For Mac
- Make sure you have installed Python 3 (Cause we are using Python 3)
- Install python3-pip
```
sudo easy_install pip
```
- install Dronekit-python
```
pip install dronekit
```

### For Linux
- Make sure you have installed Python 3 (Cause we are using Python 3)
- Install python3-pip
```
sudo apt-get install python3-pip python3-dev
```
- install Dronekit-python
```
pip3 install dronekit
```

### Documentation for control
[Ardupilot movement command](https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html)
