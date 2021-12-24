echo "export CATKIN_WS_PATH=~/catkin_ws" >> ~/.bashrc

echo "export BOOTHBOT_LOG_PATH=~/.ros/simulation_logs" >> ~/.bashrc

echo "export BACKOFFICE_LOG=~/boothbot_sim/boothbot-backoffice" >> ~/.bashrc

echo "export BOOTHBOT_DB_PATH=~/boothbot_sim/data" >> ~/.bashrc

echo "export SIMULATION_REPORT_PATH=~/boothbot_sim/reports" >> ~/.bashrc

echo "export BOOTHBOT_IMAGE=lionel-p3:3.1" >> ~/.bashrc

mkdir ~/.ros/simulation_logs -p
mkdir ~/boothbot_sim/boothbot-backoffice -p && touch ~/boothbot_sim/boothbot-backoffice/django_info.log
mkdir ~/boothbot_sim/data -p
mkdir ~/boothbot_sim/reports -p
