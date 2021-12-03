#!/usr/bin/env bash
echo "Sonar test scirpts for production"

USER=$(whoami)
TFOLDER="/home/${USER}/catkin_ws/src/boothbot/boothbot_driver/test"

echo "Front Sonar Test"
sh -c "python ${TFOLDER}/test_front_sonar.py"

echo "Front Low Sonar Test"
sh -c "python ${TFOLDER}/test_front_low_sonar.py"

echo "Rear Sonar Test"
sh -c "python ${TFOLDER}/test_rear_sonar.py"

echo " Test Done! "
