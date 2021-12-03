#!/usr/bin/python
from __future__ import print_function
import os
import time
import json
import redis
import subprocess
from functools import wraps
import uuid
import argparse

import rospy
from std_msgs.msg import Empty

from common import Logging
from common.pylaunch import PyLaunch
from common.redis_keys import ODOM_COMBINED, NAVC_STATE, RUNNING_MAP_DB_MAP

from utils import CoordConverter
from gs_state_subs import SimGSStatesSubs
from settings import (
    map_id, map_world,
    map_width, map_height,
    gs_map_pos_list, lionel_map_x_pos, lionel_map_y_pos,
    webserver_port, use_gzweb,
    use_dtu, use_twisted,
    gs_pty, cb_pty,
    boothnumber
)


def rospy_is_shutdown(fn):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        if not rospy.is_shutdown():
            fn(*args, **kwargs)
        else:
            print("Rospy already shutdown. Won't run function: {}".format(fn.__name__))
    return wrapper


class Simulation_Controller(Logging):
    """
    Simulation_Controller runs Lionel in Gazebo, and
    provide web-based GUI for Backoffice (port: 8000) and GzWeb (port 8088).

    All simulation parameters are set in local_settings.py.

    Issues
    ======
    - Fake laser scan drifts about one meter to the left, causing Lionel stuck.
    - simulation_start() continues to execute even some steps fail.
    - Only one GS is shown even more than ones are specified.
    - Need a more graceful way to end the simulation.
    """

    def __init__(self, run_gs=True, run_lionel=True):
        rospy.init_node("benchmark_simulation")
        super(Simulation_Controller, self).__init__("SIM")

        self.run_gs = run_gs
        self.run_lionel = run_lionel

        is_valid, err_msg = self._settings_is_valid()
        if is_valid is False:
            self.logerr(err_msg)
            return

        is_valid, run_local_pty, err_msg = self._virtual_ports_validation()
        if is_valid is False:
            self.logerr(err_msg)
            return
        self.run_local_pty = run_local_pty

        if self.run_lionel:
            # Topic /restart is published by goal_controller_gazebo.py
            rospy.Subscriber('restart', Empty, self.restart_cb)

        self.simulation_pkg = "boothbot_simulation"
        self.socat_window_name = "virtual_serial_port"
        self.webserver_window_name = "webserver"
        self.gazebo_win = "gazebo_web"
        self.gs_windows = []
        self.bb_window_name = "fake_bb"
        self.cmd_window_name = "command"

        # Will be reused for restarting simulation
        self.lionel_initial_x = lionel_map_x_pos
        self.lionel_initial_y = lionel_map_y_pos
        self.lionel_initial_rz = 0.0
        # If running gs on different container. Will use this fake data.
        self.gs_world_pos = (0.0, 0.0)
        self.gs_map_pos_list = gs_map_pos_list

        # When only use DTU, we cannot run MGS in simulator since we cannot share PTY
        # between GSs.
        if use_dtu and not use_twisted:
            self.logwarn("ONLY use DTU => ONLY launch as a single GS")
            self.gs_map_pos_list = self.gs_map_pos_list[:1]

        self.round = 0

        self.R = redis.StrictRedis()
        self.R.set(RUNNING_MAP_DB_MAP, map_id)
        self.is_restart = False

        # Convert map coords to Gazebo world coords
        self.converter = CoordConverter(map_width, map_height)
        # Since DTU PTY port cannot be shared between GS (it will raise error).
        # when launch gs, if `using_dtu` is True, we only start DTU for "first" GS.
        self.using_dtu = use_dtu
        self.using_twisted = use_twisted

        rospy.on_shutdown(self.end_simulation)
        self.simulation_start()

        self.rate = rospy.Rate(0.5)
        # Start ROS loop
        while not rospy.is_shutdown():
            if ((not self.run_local_pty or self.virtual_port_ok)
                and (not self.run_lionel or self.webserver_ok)
                and (not self.run_gs or self.gs_ok)
                and (not self.run_lionel or self.bb_ok)
                and not self.is_restart):
                self.loginfo("All good, simulation is running...")
            elif self.is_restart:
                self.loginfo("Restarting whole simulation, please wait...")
            else:
                self.logerr("Something wrong, simulation is stopped...")
                break

            self.rate.sleep()

        self.end_simulation()

    def _settings_is_valid(self):
        if use_dtu is False and use_twisted is False:
            return False, "Please make at least one of `use_dtu` and `use_twisted` to be `True`"
        return True, ""

    def _virtual_ports_validation(self):
        """
        Retrun: <valid>, <run start_virtual_port?>, <err msg>
        """
        run_local_pty = False
        valid = True
        err_msg = ""
        if use_dtu is True:
            # Have to confirm whether we have to run DTU `socat` locally. OR it has to be run
            # before running this script.
            if self.run_gs and self.run_lionel: # on same PC
                if not os.path.exists(gs_pty) or not os.path.exists(cb_pty):
                    run_local_pty = True
            elif (self.run_gs and not os.path.exists(gs_pty)
                  or self.run_lionel and not os.path.exists(cb_pty)):
                # running in docker container, has to make sure pty is already running on host
                valid = False
                err_msg = "Please run pty on host before running script in docker container"
        return valid, run_local_pty, err_msg

    @rospy_is_shutdown
    def _start_virtual_ports(self, wait_seconds=2):
        socat_cmd = "socat -d -d pty,raw,echo=0,link={} pty,raw,echo=0,link={}".format(
            gs_pty, cb_pty)
        self.virtual_port_ok = PyLaunch.run_cmd(self.socat_window_name, socat_cmd)
        time.sleep(wait_seconds)

    @rospy_is_shutdown
    def _start_web_server(self, wait_seconds=2):
        webserver_cmd = ("roscd boothbot && cd ../backoffice && "
                         "python manage.py runserver 0.0.0.0:{}").format(webserver_port)
        self.webserver_ok = PyLaunch.run_cmd(self.webserver_window_name, webserver_cmd)
        time.sleep(wait_seconds)

    @rospy_is_shutdown
    def _start_gazebo(self, wait_seconds=2):
        gazebo_cmd = "roscd boothbot && cd ../third_party/gzweb && npm start"
        self.gazebo_ok = PyLaunch.run_cmd(self.gazebo_win, gazebo_cmd)
        time.sleep(wait_seconds)

    @rospy_is_shutdown
    def _start_gss(self, wait_seconds=2, wait_count=100):
        # We can launch all fake GSs, as we don't use DTU in this case
        gs_launch_file = "guiding_station.launch"

        gs_world_x_pos, gs_world_y_pos = 0, 0
        gs_seqs = []

        for i, (gs_map_x_pos, gs_map_y_pos, gs_map_rz) in enumerate(self.gs_map_pos_list, 1):
            if len(self.gs_map_pos_list) == 1:
                gs_window_name = "fake_gs"
            else:
                gs_window_name = "fake_gs_{}".format(i)
            # Where we place GS in Gazebo world
            gs_world_x_pos, gs_world_y_pos = self.converter.map_to_world(gs_map_x_pos, gs_map_y_pos)
            gs_launch_args = [
                "gs_seq:={}".format(i),
                "dtu:={}".format(self.using_dtu and i == 1),
                "dtu_port:={}".format(gs_pty),
                "twisted:={}".format(self.using_twisted),
                "fake_pose:='{}, {}, {}'".format(gs_map_x_pos, gs_map_y_pos, gs_map_rz),
                "fake_hostname:={}".format(uuid.uuid4().hex),
            ]
            gs_ok = PyLaunch.launch(
                gs_window_name, self.simulation_pkg, gs_launch_file, gs_launch_args)
            if gs_ok:
                gs_seqs.append("gs_{}".format(i))
                self.gs_windows.append(gs_window_name)
                time.sleep(3.0)
            else:
                break

            if rospy.is_shutdown():
                break

        self.gs_ok = False

        gs_states_subs = SimGSStatesSubs(gs_seqs)
        # Wait at most wait_count, and each time sleep wait_seconds, for all GSs to be idle
        for i in range(wait_count):
            time.sleep(wait_seconds)
            gs_states = gs_states_subs.states.values()
            self.loginfo("Current GSs states: {}".format(gs_states))
            gs_all_idle = set(gs_states) == {'IDLE'}
            if gs_all_idle:
               self.gs_ok = True
               self.logerr("All GSs are ready after waiting {} seconds!".format((i + 1) * wait_seconds))
               break
            if rospy.is_shutdown():
                break

        # NOTE: This only show the last GS
        self.gs_world_pos = (gs_world_x_pos, gs_world_y_pos)

    @rospy_is_shutdown
    def _start_lionel(self, wait_seconds=2, wait_count=100):
        bb_launch_file = "boothbot_sim.launch"
        # Where we place Lionel in Gazebo world
        lionel_world_x_pos, lionel_world_y_pos = self.converter.map_to_world(
            lionel_map_x_pos, lionel_map_y_pos)
        # NOTE: This only show the last GS
        gs_world_x_pos, gs_world_y_pos = self.gs_world_pos
        bb_launch_args = [
            "map_id:={}".format(map_id),
            "map_world:={}".format(map_world),
            "boothnumber:={}".format(boothnumber),
            "x_pos:={}".format(lionel_world_x_pos),
            "y_pos:={}".format(lionel_world_y_pos),
            "gs_x_pos:={}".format(gs_world_x_pos),
            "gs_y_pos:={}".format(gs_world_y_pos),
            "using_dtu:={}".format(self.using_dtu),
            "cb_dtu_port:={}".format(cb_pty),
            "using_twisted:={}".format(self.using_twisted),
        ]
        self.bb_ok = PyLaunch.launch(
            self.bb_window_name, self.simulation_pkg, bb_launch_file, bb_launch_args)

        # Wait at most wait_count, and each time sleep wait_seconds, for Boothbot to be idle
        for i in range(wait_count):
            time.sleep(wait_seconds)
            state = self.R.get(NAVC_STATE) or ''
            self.loginfo("Current NAVC state: {}".format(state))
            # Very coarse comparision, but good enough as RUN takes some time
            if state == "IDLE":
               self.logerr("NAVC is ready after waiting {} seconds!".format((i + 1) * wait_seconds))
               break
            if rospy.is_shutdown():
                break

    @rospy_is_shutdown
    def _initial_pose(self, wait_seconds=2, wait_count=100):
        time.sleep(wait_seconds)
        initialpose_args = "{} {} {}".format(
            self.lionel_initial_x, self.lionel_initial_y, self.lionel_initial_rz)
        initialpose_cmd = "rosrun boothbot_control initialpose.py {}".format(initialpose_args)
        PyLaunch.run_cmd(self.cmd_window_name, initialpose_cmd)

        # Wait at most wait_count, and each time sleep wait_seconds, for Initial pose done
        for i in range(wait_count):
            if i % 50 == 0:
                PyLaunch.run_cmd(self.cmd_window_name, initialpose_cmd)

            time.sleep(wait_seconds)
            odom_x, odom_y, odom_rz = json.loads(self.R.get(ODOM_COMBINED) or "[-10000, -10000, 0]")
            self.loginfo("Current pos: {}".format((odom_x, odom_y, odom_rz)))
            # Very coarse comparision, but good enough as RUN takes some time
            if (abs(odom_x - self.lionel_initial_x)) < 0.2 and \
               (abs(odom_y - self.lionel_initial_y)) < 0.2 and \
               (abs(odom_rz - self.lionel_initial_rz)) % 180.0 < 36: # In case 180 and -180
                self.logerr("Initial pose done after waiting {} seconds!".format((i + 1)* wait_seconds))
                break
            if rospy.is_shutdown():
                break


    @rospy_is_shutdown
    def _start_run(self):
        run_cmd = "rostopic pub -1 /boothbot_controller/cmdword std_msgs/String RUN"
        PyLaunch.run_cmd(self.cmd_window_name, run_cmd)

    def simulation_start(self):
        if self.run_local_pty:
            # Step 1. Run virtual port (This is needed even using_dtu is False)
            self._start_virtual_ports()

        if self.run_lionel:
            # Step 2. Run Backoffice
            self._start_web_server()

            # Step 3. Run GzWeb
            # FIXME: have no idea why gzweb does not render the hall world
            # if use_gzweb is True:
            #     self._start_gazebo()

        if self.run_gs:
            # Step 4. Launch GSs
            self._start_gss()

        if self.run_lionel:
            # Step 5. Launch Lionel
            self._start_lionel()

            # Step 6. Initial pose
            self._initial_pose()

            # Step 7. Run
            self._start_run()

            # Step 8. Restart gzweb to display the hall world
            PyLaunch.shutdown(self.gazebo_win)
            time.sleep(0.5)
            # if use_gzweb is True:
            #     self._start_gazebo()

            self.round += 1
            self.logerr("Simulation round: {}".format(self.round))
    def restart_cb(self, msg):
        # Only lionel is able to run CB
        if self.run_lionel is False:
            return
        
        self.lionel_initial_x, self.lionel_initial_y, self.lionel_initial_rz \
            = json.loads(self.R.get(ODOM_COMBINED) or "[0, 0, 0]")

        self.is_restart = True

        PyLaunch.shutdown(self.bb_window_name)

        time.sleep(2)

        # Step 5. Launch Lionel
        self._start_lionel()

        # Step 6. Initial pose
        self._initial_pose()

        # Step 7. Run
        self._start_run()

        # Restarted
        self.is_restart = False

        self.round += 1
        self.logerr("Simulation round: {}".format(self.round))

    def end_simulation(self):
        """
        Stop simulation processes by killing tmux panels.
        """
        self.R.delete(RUNNING_MAP_DB_MAP)
        self.logerr("Simulation is shutting down...")

        # Ending reversely comparing to start launch windows
        if self.run_lionel:
            PyLaunch.shutdown(self.bb_window_name)
        if self.run_gs:
            for w in self.gs_windows:
                PyLaunch.shutdown(w)
        if self.run_lionel:
            if use_gzweb is True:
                PyLaunch.shutdown(self.gazebo_win)
            PyLaunch.shutdown(self.webserver_window_name)
        if self.run_local_pty:
            PyLaunch.shutdown(self.socat_window_name)

        self.logerr("Total simulation round: {}".format(self.round))
        self.logerr("...Bye!")


if __name__ == "__main__":

    parser = argparse.ArgumentParser("Which platform run simulator?")
    parser.add_argument('platform', type=str, nargs='?', default="",
                        help='gs/lionel/both, if no input, default is both on same PC')

    args = parser.parse_args()
    pf = args.platform.lower()

    if pf not in ["gs", "lionel", "", "both"]:
        print('Please make sure platform is "gs"/"lionel"/both"/""(empty) ')
    else:

        run_gs = run_lionel = True
        if pf == "gs":
            run_lionel = False
            print("Will only run GS")
        elif pf == "lionel":
            run_gs = False
            print("Will only run Lionel")
        else:
            print("Will run GS and Lionel")

        try:
            Simulation_Controller(run_gs=run_gs, run_lionel=run_lionel)
        except rospy.ROSInterruptException:
            pass
