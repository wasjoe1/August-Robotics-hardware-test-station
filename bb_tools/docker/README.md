## How to setup on your pc?
### Pre-requisite
* Install [docker](https://docs.docker.com/get-docker/)
* Install [docker-compose](https://docs.docker.com/compose/install/)
* This env is made for boohtbot v3.0+

### 1. Update submodule
```bash
git submodule init && git submodule update
```

### 2. Ensure the following environment & folder **exist**
1. Catkin workspace path; This Should be the catkin_ws that you stored you boothbot repo
    `echo "export CATKIN_WS_PATH=~/catkin_ws" >> ~/.bashrc`
2. Place to store boothbot simulation's log; Could be wherever you want
    `echo "export BOOTHBOT_LOG_PATH=~/.ros/log" >> ~/.bashrc`
3. Simulation Data Storage; Could be wherever you want
    `mkdir ~/boothbot_sim_data && echo "export BOOTHBOT_DB_PATH=~/boothbot_sim_data" >> ~/.bashrc`
4. Place to store simulation report; Could be wherever you want
    `mkdir ~/boothbot_sim_reports && echo "export SIMULATION_REPORT_PATH=~/boothbot_sim_reports" >> ~/.bashrc`

### 3. Change the path to the path of boothbot repo on your pc in line 12 in `boothbot/docker/docker-compose.yml`
```xml
volumes:
    - "/etc/timezone:/etc/timezone:ro"
    - "/etc/localtime:/etc/localtime:ro"
    - <path to boothbot repo>:/home/augbooth/catkin_ws/src/boothbot:rw
    #e.g.-/home/augbooth/catkin_ws/src/boothbot:/home/augbooth/catkin_ws/src/boothbot:rw
```

### 4. Docker image perparation
1. Go to `docker/` directory under `augustbot-tools` repo.

    `#e.g. cd /home/augbooth/catkin_ws/src/augustbot-tools/bb_tools/docker`

2. Build Dockerfile to create the image `lionel`.

   `docker build -t lionel .`

### 5. Run below command in `augustbot-tools/bb_tools/docker`
```bash
docker-compose up -d
```

### 6. Perpare the Map as usual
1. Open a web browser and login to LIONEL's GUI
    `#e.g. http://i7-0000.local`
    or
    `#e.g. http://localhost:8000`

2. Create a new map or select an existing map; Remember the map_id as it will be used later on


### 7. Run the Simulation
1. Go into the `boothbot_main` container in a new terminal
    `docker exec -it boothbot_main bash`

2. Go to the `/boothbot_simulation/script` directory under `augustbot-tools` repo.
    `simulation`
    or
    `/home/augbooth/catkin/src/augustbot-tools/boothbot_simulation/script`

3. Execute the `run_simulation.py` script
    `python run_simulation.py`
    then follow the instruction displayed on terminal

    **NOTE: This could only used in the boothbot_main container, as it involved some environment varibles that has only been load into boothbot_main container**
### 8. Generate a test report
1. Excute the `generate_report.py` script
    file path should be the output of the pervious step
    `#e.g. /home/augbooth/Simulation_Reports/21-12-08-task-I4KFK0_imu-wrong-data-effect/rosout-20211208142351.log`

    or any rosout.log file in log folder

    **NOTE: This could only used in the boothbot_main container, as it involved some environment varibles that has only been load into boothbot_main container**
### 9. Terminate the docker container
1. Go to the terminal for launching `docker-compose up -d`.

2. Stop all Docker containers by running `docker-compose down`.
    **NOTE: This is necessary to ensure that the log for each simulation is stored separately. The report generator only summarizes the first job in the log.**





### Ensure the config/setting files **exist**
#### Make sure the files below **exist**:
1. `boothbot/common/scripts/common/device_name.yaml`
2. `boothbot/backoffice/backoffice/lionel/local_settings.py`
#### if not, you can find them in `boothbot/docker/config`
