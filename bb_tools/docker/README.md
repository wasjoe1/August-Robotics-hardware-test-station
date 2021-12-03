## How to setup on your pc?
### Pre-requisite
* Install [docker](https://docs.docker.com/get-docker/)
* Install [docker-compose](https://docs.docker.com/compose/install/)
* This env is made for boohtbot v3.0+
 
### 1. Update submodule
```bash
git submodule init && git submodule update
```

### 2. Change the path to the path of boothbot repo on your pc in line 12 in `boothbot/docker/docker-compose.yml`
```xml
volumes:
    - "/etc/timezone:/etc/timezone:ro"
    - "/etc/localtime:/etc/localtime:ro"
    - <path to boothbot repo>:/home/augbooth/catkin_ws/src/boothbot:rw
```

### 3. Run below command in `boothbot/docker`
```bash
docker-compose up -d
```

### 4. Ensure the config/setting files **exist**
#### Make sure the files below **exist**:
1. `boothbot/common/scripts/common/device_name.yaml`
2. `boothbot/backoffice/backoffice/lionel/local_settings.py`
#### if not, you can find them in `boothbot/docker/config` 

### 5. Run below command to access the container
```bash
docker exec -it lionel /bin/bash
```

### 6. catkin build
We have a script to do the catkin build, run it by
```bash
/build.bash
```

### 7. Source the setup.bash built by catkin
```bash
source ~/catkin_ws/devel/setup.bash
```
If you want you put this command in ~/.bashrc, run this
```bash
grep -qxF "source ~/catkin_ws/devel/setup.bash" ~/.bashrc || echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc;
```

### 8. Migrate db
```bash
cd ~/catkin_ws/src/boothbot/backoffice && ./manage.py migrate
```

### 9. Start backoffice
```bash
cd ~/catkin_ws/src/boothbot/backoffice && ./manage.py runserver 0.0.0.0:8000
```
