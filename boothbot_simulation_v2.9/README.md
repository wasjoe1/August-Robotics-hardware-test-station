`boothbot_simulation` is based on GzWeb to run our Boothbot in a simulation server. It will repeatly perforam marking on a preset map (currently a testmap in HALL C2 of LLVC).

Usage
===
Before staring running, set the Twisted communication environment variables. Add following two lines in `~/.bashrc` file. The IP is Lionel large PC IP, and `GS_LAN_PORT` is GS port which is connecting router. We add below two since we are running simulator on same PC. ALL is localhost.

```
export TC_HOST_IP=127.0.0.1 # this is your large PC IP
export GS_LAN_PORT=lo # this is GS pc LAN port
export PTY_DTU=1 # when using PTY as DTU.
```

To run simulation, login the simulation server, open a tmux session, then run `roslaunch boothbot_simulation run_benchmark.py`, then wait for a couple of minutes untill all simulation processes get started. NOTE: please make sure `roscore` is running in the first place.

View Backoffice via `simulation-server.local:8000`, and view GzWeb via `simulation-server.local:8080`.

To stop a running simulation, first stop the aforementioned node, then kill the tmux session `bb_session`.

Buiding GzWeb
===
GzWeb is a web client of the Gazebo simulator. In our case, it's installed in `~/catkin_ws/src/boothbot/third_party/gzweb` on the simulation server.

The most challenging step in deploying simulation is to build GzWeb, which run atop nodejs and depends on many old/deprecated versions of npm packages.

To compile `node-gyp`, `node` major version must be v10.

The version of the `karma` package in `package.json` is very old, and depends on deprecated node version. So this version must be changed to `^2.0.0`.

See the official tutorial on how to build GzWeb: http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb.

Other Dependancies
===
- Install `socat` by `sudo apt install socat`

Known Issues
===
- Lionel may get stuck in some postions, and cannot recovery from it. If such situation happens, you can stop the simulation, then run it again. Disabling some trouble-making corners may help.
- Directly invoking goal_controller.py for simulation causes Lionel stuck after first mark.
- Simulations consumes almost 100% of compute and RAM.
- [Very rare] Sometimes simulator fails to create virtual ports on develop machine. This happens if the simulation session is killed accidentally. Restarting PC can fix this.
- [Extremely rare] Some GS's position becomes [0, 0, 0] after restarting.

Future Features
===
- Generate Gazebo worlds from dxf files or database maps.
- Show more than one GSs on map details page.


NOTE
===

When running MGS with PTY, one of GS may raise following error. The reason the DTU error raises since we are using same port for MGS DTU. 

SO, we only use DTU on 1st GS only.

```
Exception in thread DTU:
Traceback (most recent call last):
 File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
   self.run()
 File "/root/catkin_ws/src/boothbot/common/scripts/common/dtu/transceiver.py", line 62, in run
   self.poll()
 File "/root/catkin_ws/src/boothbot/common/scripts/common/dtu/transceiver.py", line 72, in poll
   start_sign = self._port.read(1)
 File "/usr/lib/python2.7/dist-packages/serial/serialposix.py", line 495, in read
   raise SerialException('device reports readiness to read but returned no data (device disconnected or multiple acc
ess on port?)')
SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on por
t?)
```
