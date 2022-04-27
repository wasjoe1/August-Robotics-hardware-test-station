
# export BOOTHBOT_IS_SIMULATION=false
export BOOTHBOT_IS_SIMULATION=true
echo "Running simulation? $BOOTHBOT_IS_SIMULATION"

export BOOTHBOT_LIDAR_TYPE=YD-TX8
# export BOOTHBOT_LIDAR_TYPE=YD-X4
# export BOOTHBOT_LIDAR_TYPE=LiDAR
# export BOOTHBOT_LIDAR_TYPE=RP-A3
echo "Set LiDAR type to $BOOTHBOT_LIDAR_TYPE"

# export BOOTHBOT_BASE_TYPE=HangFa
export BOOTHBOT_BASE_TYPE=DJI
echo "Set Base type to $BOOTHBOT_BASE_TYPE"
