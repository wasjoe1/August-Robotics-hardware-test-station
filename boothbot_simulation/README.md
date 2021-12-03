# Instructions for Running the Simulator
Last updated on: Nov. 25, 2021.

## 1. Setting Up

### Docker
1. Go to `docker/` directory.
1. Build Dockerfile to create the image `lionel`.

   `docker build -t lionel .`

### Backoffice / Boothbot Settings
1. Copy the file `docker/config/device_name.yaml` into the `common/scripts/common/` directory.
1. Copy the file `docker/config/local_settings.py` into the  `backoffice/backoffice/lionel/` directory.

## 2. Launching Docker Images 
1. Go to `docker/` directory.
1. Update the settings (e.g., the path to the catkin workspace) in the `.env` file if necessary.
1. Bringup Docker images specified in `docker-compose.yml`.

   `docker-compose up -d`

1. Wait until the `roscore` finishes building the catkin workspace and starts running. **NOTE: As part of this process, the existing `devel`, `build` and `logs` directory will be cleaned!**

1. Wait until the `backoffice` starts running.

1. Wait until `bringup_fake_gs`, `bringup_hardware` and `boothbot_main` start running.

## 3. Running the Simulation

### 1. Real Map
1. Go to `http://localhost:8000`.
1. Prepare real map as usual.

### 2. Fake Drivers
1. Enter the `bringup_hardware` container (in a new terminal).

   `docker exec -it bringup_hardware /bin/bash`

1. Launch fake drivers.

   `roslaunch boothbot_simulation bringup_fake_hardware.launch`

1. Wait until all fake drivers are running (i.e., until `Sonar_D: Sonars are initialized!`).

### 3. Boothbot Main (with Fake Camera Beacon)

1. Enter the `boothbot_main` container (in a new terminal).

   `docker exec -it boothbot_main /bin/bash`

1. Launch `boothbot_main`. Be sure to set the values of `fake_cb`, `using_dtu`, `fake_chassis_usb` and `fixed_lan_ip`.

   `roslaunch boothbot_main boothbot_main.launch fake_cb:=true using_dtu:=false fake_chassis_usb:=true fixed_lan_ip:=true`

1. Wait until everything is connected, e.g.:

   `[WARN] [1637723375.247317]: task_manager.GOTO_MRK_C: Init server connected!!`\
   `[WARN] [1637723375.348603]: task_manager.NAV_C: Nav server connected!!`\
   `[WARN] [1637723375.449732]: task_manager.MRK_C: Marking server connected!!`

**NOTE: If errors are problems occur at this stage, try ending the simulation and run `docker-compose up` again. This will re-build all `src` files.**

### 4. Fake GS
1. Open `boothbot_simulation/launch/bringup_fake_gs.launch`.
1. Adjust the number of fake guiding stations and their values for `gs_seq`, `fake_hostname` and `fake_pose` as necessary.

1. Enter the `bringup_fake_gs` container (in a new terminal).

   `docker exec -it bringup_fake_gs /bin/bash`

1. Launch fake guiding station.

   `roslaunch boothbot_simulation bringup_fake_gs.launch`

1. Wait until the connection is established (i.e., until `GSClientP: sending connection made`). If boothbot_main is running correctly, it will also report that a guiding station has been detected. E.g.:

   `[WARN] [1636280728.908226]: Goal Controller: Add gs_1 in gs list`\
   `[WARN] [1636280728.908398]: cb_redis: Add gs_1 in gs list`\
   `[WARN] [1636280728.911905]: map_pcd_driver: Added/Updated gs_1 in gs list`

### 5. Simulation

1. Enter the `boothbot_main` container in a new terminal (or any other container running the `lionel` image).

   `docker exec -it boothbot_main /bin/bash`

1. Go to `boothbot_simulation/scripts` directory.
1. Execute `run_simulation.py` script. Ensure that the inputs for `map_id` and `gs_id_list` correspond to the ones used for launching the ROS nodes. **NOTE: The simulation can be monitored as usual using the terminal and web interface.**

   `./run_simulation.py`

1. Execute `generate_report.py` script to generate a report. For convenience, symbolic links to the simulation logs can be found in `boothbot_simulation/reports` directory. `rosout-latest.log` will always point to the latest log generated, while `rosout-[DATETIME]` will point to the log for the simulation for that given date-time.

   `./generate_report.py`

## 4. Ending the Simulation
1. Go to the terminal for launching `docker-compose up -d`.

1. Stop all Docker containers by running `docker-compose down`. **NOTE: This is necessary to ensure that the log for each simulation is stored separately. The report generator only summarizes the first job in the log.**