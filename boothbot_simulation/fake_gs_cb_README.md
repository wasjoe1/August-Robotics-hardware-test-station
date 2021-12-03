# Instructions for Running the Fake GS and CB
Last updated on: Oct. 27, 2021.

## Setting Up

### 普通 Lionel，非 docker
1. 更新 `boothbot` 代码库

2. 设置 ～/.bashrc ，新增代码：  
   `export TC_HOST_IP=127.0.0.1 # this is your large PC IP`  
   `export GS_LAN_PORT=lo # this is GS pc LAN port`  
   `export PTY_DTU=1 # when using PTY as DTU.`  
   `alias rungs="roslaunch boothbot_simulation fake_gs.launch"`  
   `alias runfakecbmian="roslaunch boothbot_simulation boothbot_main_fake_cb.launch"`  

3.  然后使用 tmux，按顺序 `roscore`, `runhw`, `runfakecbmian`, `rungui`, `rungs` 就可以了

### docker Lionel
1. 注意：由于 `docker` 和 `boothbot-configs` 并非是代码仓库，没有代码回溯功能，修改前请先备份`docker`和 `boothbot-configs` 的文件夹，或者使用注释代码和新增新代码去修改代码

2. 更新 `boothbot` 代码库

3. 打开 `docker-compose.yaml` ，参考其他 `ros` 的容器，增加一个容器 `fake_gs`，用于打开 `fake_gs`

4. 打开 `boothbot-configs/scripts/boothbot_main.bash`, 将打开 `launch` 相关的代码修改成下面：

   `export TC_HOST_IP=127.0.0.1 # this is your large PC IP`  
   `export GS_LAN_PORT=lo # this is GS pc LAN port`  
   `export PTY_DTU=1 # when using PTY as DTU.`  
   `source /home/augbooth/catkin_ws/devel/setup.bash;`  
   `stdbuf -o L roslaunch --wait boothbot_simulation boothbot_main_fake_cb.launch;`  

5. 参考 `boothbot-configs/scripts/boothbot_main.bash`的代码，新建 `boothbot_fake_gs.bash`，主要代码如下：

   `export TC_HOST_IP=127.0.0.1 # this is your large PC IP`  
   `export GS_LAN_PORT=lo # this is GS pc LAN port`  
   `export PTY_DTU=1 # when using PTY as DTU.`  
   `source /home/augbooth/catkin_ws/devel/setup.bash;`  
   `stdbuf -o L roslaunch --wait boothbot_simulation fake_gs.launch;`  

6. 然后在 `docker` 目录下 `docker-compose down` , `docker-compose up`，如果 `fake_gs` 无法连接，请尝试 `docker container stop fake_gs` , 然后 `docker-compose up fake_gs`  

7. 如果 `fake_gs` 有问题，请尝试使用下面简单的修改代码方式解决  
`boothbot_nav/scripts/clients/twisted_comm.py` 倒数第二行修改成 `reactor.listenTCP(TWISTED_COMM_PORT, ServerFactory(), interface="127.0.0.1")`  
`guiding_beacon/scripts/guiding_beacon/gs_twisted_comm.py` 的 `main` 函数里面，注释 `time.sleep(20.)` 并在同一级增加代码 `lionel_host_ip = "127.0.0.1"`，这样子程序在下一个loop就会跳出循环，然后连接 `twisted` 服务器  
