# TODO confirm where we wanna to save `device_name.yaml`
DN="`rospack find common`/scripts/common/device_name.yaml"
if [ -e $DN ]; then
  ROBOT_NAME=`grep -o -E -e '(\w*-)+\w*' $DN`
  echo "ROBOT_NAME    :" $ROBOT_NAME
fi

if [ -n "$1" ]; then
  if [ -n "$2" ]; then
    ROS_IP=$2
    ROS_MASTER_URI=http://$1:11311
    echo "ROS_MASTER_URI:" $ROS_MASTER_URI
    echo "ROS_IP        :" $ROS_IP
    # source will export the env automatically
    # export ROS_IP
    # export -n ROS_HOSTNAME

  else
    ROS_HOSTNAME=`uname -n`.local

    if [[ $1 -ge 1000 ]]; then
      ROS_MASTER_URI=http://j1900-$1.local:11311
    elif [[ $1 -ge 100 ]]; then
      ROS_MASTER_URI=http://j1900-0$1.local:11311
    elif [[ $1 -ge 10 ]]; then
      ROS_MASTER_URI=http://j1900-00$1.local:11311
    else
      ROS_MASTER_URI=http://j1900-000$1.local:11311
    fi

    echo "ROS_MASTER_URI:" $ROS_MASTER_URI
    echo "ROS_HOSTNAME  :" $ROS_HOSTNAME
    # export ROS_HOSTNAME
    # export -n ROS_IP
  fi

else
  ROS_HOSTNAME=`uname -n`.local
  ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
  echo "ROS_MASTER_URI:" $ROS_MASTER_URI
  echo "ROS_HOSTNAME  :" $ROS_HOSTNAME
  # export ROS_HOSTNAME
  # export -n ROS_IP
fi
# export ROS_MASTER_URI
