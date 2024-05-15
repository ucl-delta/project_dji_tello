# Project DJI Tello

Project for launch DJI Tello

## Install

This assumes the installation of aerostack and also the `as2_platform_tello` package

## Launch


```bash
./launch_as2.bash
```

You can use the teleop panel

You first need to install pysimplegui: `pip install pysimplegui==4.60` 

```bash
./launch_as2.bash -t
```
Note that the settings on the teleop panel allow you to change the requested speed. 

## Configuration

The `config/platform.yaml` contains the ip address to connect to the tello

Thie `motion_controller_plugin.yaml` contains the pid controller tuning

## Known Issues

1. Bench Testing - While not in flight the DJI Tello will constantly set its height to zero. This is a known effect as it tries to understand where the takeoff zero is before it takes off
2. Odometry - Something is not quite right with the odometry as position control is a bit wonky. 
3. When testing with teleop, it sometimes loses directionality. 
4. By default this uses speed control not sure how accurate it is.  
5. You may need to restart the tello each time you stop this launch script (or restart the tello_platform node). Ongoing issue in which you can't reconnect with the Tello for some reason. Linked to this, sometimes if you connect to soon after the Tello has turned on, there's a timing issue causing a connection just after ctrl+c of the tello_platform node causing you to have to restart everything again. (If the alphanumeric display doesn'y show anything, likely that this has happened)

Note with TMUX, to change window you first press Ctrl+b, then select 0-5. 

