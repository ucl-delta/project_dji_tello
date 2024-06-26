# Project DJI Tello

Project for launch one or more DJI Tellos in reality and testing applications in simulation. 

## Install

This assumes the installation of aerostack and also the `as2_platform_tello` package

For simulation this will require the installation of a suitable Gazebo - see aerostack documentation for more details. 

You will likely want to use this with external positioning (using mocap4ros2 protocol) as internal positioning is terrible. We have tested with optitrack. 

## Launching Single Tello

### Setup Tello

The configuration files for connecting to a tello are located in `config/tellox`. By default tello1 will launch. You will want to ensure that the IP address of the tello is correct, and that the ports chosen for state and video streaming are not in use. 

To check your connection with the tello, you can use the `udp_connect_tello.py` script and give it string commands as per the SDK2.0 or SDK3.0 commands. Remember the first command you should give it is `command` to initialise. 

To find the IP address of a tello you can use the following methods:
- If it is a standard tello, and you are connected to the Tello's broadcast network the default IP address will be 192.168.10.1
- You can set the tello into AP mode (where it will connect to an existing network) by issuing the `ap <SSID> <PASSWORD>` using the python script. 
- Once on your network, it will be assigned an IP address by your router (you may want to give it a static DHCP IP by logging on your router and adding it to the list of static IPs). 

If using external positioning, such as optitrack, you should also setup the `config/telloX/state_estimator_mocap.yaml` so that it is tracking the correct drone.  

### Launching

First put a battery in the tello and tun it on. After startup it should start fast blinking yellow.

You must first source your aerostack installation (and also your optitrack workspace if neccesary)

Then navigate the the root directory of this repository and run the following command:

```bash
./launch_as2.bash
```

> The default vehicle will be `tello1`

This will start a number of ROS2 nodes running in the background, and drop you into a monitoring window displaying the current status of the tello. 

If using optitrack, you will need to change the estimator plugin to use `pose_mocap`:

```bash
./launch_as2.bash -e pose_mocap
```

### Navigating around and viewing information

Once you have launched, it will open up a number of tmux sessions in the background. Tmux is a utility which allwos you to run multiple programs in the same terminal window. 

You can return from the default alphanumeric viewer to the terminal by detaching from the current tmux session using `ctrl+b, d` 

> Note this is not pressing all the keys at the same time. This is first pressing `ctrl+b`, then pressing `d`

> Note also that exiting by ctrl+c will just stop whatever's running and not actually detach you. This might beuseful if you want to restart whateever process you have running in that session.

From the terminal you can see the currently running tmux sessions using `tmux ls` 

You can attach to any one of them using the command `tmux a -t <session_name>`. For example to connect back to the alphanumeric viewer you can use `tmux a -t alpha`. 

Either detaching from the current session or opening a new terminal (with everythin sourced), you can debug the tello connection by connecting to the tello1 session: `tmux a -t tello1`. 

This will present you with another tmux session, although this time the session has multiple tabs for each system in the drone control stack
- platform launch
- state estimator
- motion-controller
- motion behaviours

To scroll through the different pages use `ctrl+b`, page_id. E.g. `ctrl+b`, `0` for the first tab. 

Each tab is designed to be independent, so you can change configurations and re-run the particular ros2 launch command in each tab without it affecting each other.

> Note: often the tello connection in the `platform` tab may have issues. It is worth keeping an eye on that process if things aren't quite working. YOu may need to restart it to re-connect to the tello. This is also handy if you need to change battery as you can simply restart the command in this tab, instead of having to restart the entire stack. 

### Launch Options:

#### Teleop

You can use the teleop panel

You first need to install pysimplegui: `pip install pysimplegui==4.60` 

```bash
./launch_as2.bash -t
```
Note that the settings on the teleop panel allow you to change the requested speed and positioning.

#### Foxglove Bridge

We often use the foxglove visualisation software for monitoring flights. This requires the use of the foxglove bridge. Use the `-f` option to automatically start the bridge

#### Record ROSBAG

Use the `-r` option to automatically start a rosbag recording

### Running a script

The recommended method currently is to start a second terminal with all the correct things sourced to run your control script. Then you can simply run your python script which uses the as2 python api

```bash
python3 mission.py
```

This basic mission allows you to manually fly the tello to different locations

### Stopping Everything

To stop everything from running, you just need to run the stop script in the root of this project. 

```bash
./stop.bash
# or
./stop.bash all
```

> Will stop all alive tmux sessions

To only stop the drone, you can supply the tmux session name

```bash
./stop.bash tello1
```

## Launching Multiple Tellos

Similarly to before, make sure you have setup each of the tellos in config with their own configuration files, and you have the correct IP addresses, and ensure that none of the ports clash with each other. 

> Note, we have tested with multiple drones connected in AP mode to a single router, and the controlling computer on the same network. Not sure how this would change if control computer had multiple wifi dongles with individual connections. 

Then place the tellos into the flying space and turn them on. 

To launch aerostack with multiple tellos, you can specify the drone names in a comma separated list:

```bash
./launch.bash -e mocap_pose -n tello1,tello2
```

This will run one flight stack for each drone in its own tmux session. 

> Remember to add extra flags as needed. Anding `-t` for teleop will enable the teleop panel to control both drones (Although performance is a bit flaky)

Then to control multiple tellos, you will need to have a python script capable of coordinating multiple drones. The `mission_swarm.py` and `mission_robodome.py` will both allow you to by passing in a list of namespaces of drones:

```bash
python3 mission_swarm.py --drone_ns tello1 tello2
```

Again use `./stop.bash` to stop everything. 

## Launch Simulation

This repository also enables simulation to real testing through aerostack. It contains the elements required to run via a gazebo simulation. To run with a gazebo simulation simply pass the `-s` flag in.

```bash
# for launching one drone `tello1`
./launch_as2.bash -s 
# for launching multiple drones
./launch_as2.bash -s -n tello1,tello2,tello3
```

> Note: There is no need to pass the `-e pose_mocap` when using simulation, it is set to use the `ground_truth` from the gazebo simulation

This command will start up the gazebo simulator, as well as all the elements as expected. 

### Modifying the world

Afaik, aerostack doesn't enable dynamic spawning of drone just yet, as a result it just spawns the drones from a hardcoded `word.json` file where the names must match up to the drone_names used within launch_as2. To enable the use of different numbers of drones, we have a script `generate_world.py` which will generate the world file. This script is automatically called within `launch_as2`. 

All this script does is add to the `drones` array provided within a `world_base.json`. Within the world_base json file you can then add objects as normal within aerostack. Therefore you can modify the world_base to create the base world you want to simulate. 

> Note: The drones are spawned in a line along the x axis, just be aware not to spawn any objects on top of them! 

## Author

Mickey Li (mickey.li@ucl.ac.uk)
Aerostack Team