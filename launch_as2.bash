#!/bin/bash

usage() {
    echo "  options:"
    echo "      -r: record rosbag"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap_pose]"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is tello"
}

# Arg parser
while getopts "e:n:rt" opt; do
  case ${opt} in
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    e ) 
      estimator_plugin="${OPTARG}"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
estimator_plugin=${estimator_plugin:="raw_odometry"}  # default ign_gz
drone_namespace=${drone_namespace:="tello1"}


tmuxinator start -n ${drone_namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${drone_namespace} \
    estimator_plugin=${estimator_plugin} &
echo "Background processes started ${drone_namespace} running ${estimator_plugin}..."
wait

if [[ ${record_rosbag} == "true" ]]; then
    tmuxinator start -n rosbag -p tmuxinator/rosbag.yaml \
        drones=${drone_namespace} &
    wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
    tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yaml \
        drone_namespace=${drone_namespace} &
    wait
fi

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
  tmuxinator start -n mocap -p tmuxinator/mocap.yml &
  wait
fi

# Attach to tmux session
tmux attach-session -t ${drone_namespace}:alphanumeric_viewer
