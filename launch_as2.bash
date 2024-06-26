#!/bin/bash

usage() {
    echo "  options:"
    echo "      -r: record rosbag"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap_pose]"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespaces, default is tello1 - can also pass a comma separated list e.g tello1,tello2"
    echo "      -f: launch foxglove bridge"
}

# Arg parser
while getopts "e:n:rtf" opt; do
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
      drone_namespaces="${OPTARG}"
      ;;
    f )
      foxglove_bridge="true"
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
drone_namespaces=${drone_namespaces:="tello1"}
foxglove_bridge=${foxglove_bridge:="false"}

# Read drone namespaces as comma separated array. 
IFS=',' read -ra namespace_array <<< "$drone_namespaces"

for drone_namespace in "${namespace_array[@]}"; do
  tmuxinator start -n ${drone_namespace} -p tmuxinator/aerostack2.yaml \
      drone_namespace=${drone_namespace} \
      estimator_plugin=${estimator_plugin} &
  echo "Background processes started ${drone_namespace} running ${estimator_plugin}..."
  wait
done

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yaml \
      drone_namespace=${drone_namespaces} &
  wait
fi

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
  tmuxinator start -n mocap -p tmuxinator/mocap.yml &
  wait
fi

if [[ ${record_rosbag} == "true" ]]; then
    tmuxinator start -n rosbag -p tmuxinator/rosbag.yaml \
        drones=${drone_namespaces} &
    wait
fi

if [[ ${foxglove_bridge} == "true" ]]; then
    tmuxinator start -n foxglove -p tmuxinator/foxglove.yaml &
    wait
fi


tmuxinator start -n alphanumeric -p tmuxinator/alphanumeric.yaml drone_namespaces=${drone_namespaces} &
wait

# Attach to tmux session
tmux attach-session -t alpha:alphanumeric_viewer
