#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

"""
The navigation controller client

This is the client for other controller to control navigation module.

"""

import rospy
logger = rospy
from actionlib import SimpleActionClient, SimpleGoalState as SS, GoalStatus

from boothbot_nav.settings import CONNECT_TIMEOUT
from boothbot_common.ros_logger_wrap import ROSLogging
from boothbot_nav.states import NavigationServerStates as MS, NavigationServerCommand as CS
from boothbot_msgs.ros_interfaces import MODULES_NAV_STATUS, MODULES_NAV_SRV_CMD, \
    MODULES_NAV_ACT_NAV, MODULES_GS_HUB_PDO
import boothbot_msgs.msg as bbmsgs

CLIENT_NAME = 'NAV_C'

class NavigationClientROS(ROSLogging):
    __slots__=[
        'is_connected',
        'act_cli',
        '_sub_status',
        '_msg_status',
        '_srv_cmd_fn',
    ]
    def __init__(self, parent_prefix=''):
        '''
        A client that control the self designed navigation controller server
        '''
        super(NavigationClientROS, self).__init__('{}.{}'.format(parent_prefix, CLIENT_NAME))
        self.is_connected = False
        self._msg_status = MODULES_NAV_STATUS.type()
        self._msg_status.state = MS.INIT.name

    #######################################################################
    #  Client's API
    #######################################################################
    #################
    ##  Status
    @property
    def state(self):
        return MS[self._msg_status.state]

    @property
    def errorcodes(self):
        return self._msg_status.errorcodes

    def is_active(self):
        return self.act_cli.simple_state in (SS.PENDING, SS.ACTIVE)

    def is_done(self):
        return self.act_cli.simple_state == SS.DONE

    # def is_paused(self):
    #     return self.state == MS.PAUSED

    def is_succeeded(self):
        res = self.act_cli.get_result()
        if res is None:
            return False
        return res.result == bbmsgs.NavigationResult.SUCCEEDED

    def is_failed(self):
        res = self.act_cli.get_result()
        if res is None:
            return False
        return res.result == bbmsgs.NavigationResult.FAILED

    def get_result(self):
        return self.act_cli.get_result()

    #################
    ## Operation
    def connect(self, timeout=CONNECT_TIMEOUT):
        if self.is_connected:
            return True

        # subscribe status
        self._sub_status = rospy.Subscriber(MODULES_NAV_STATUS.name,
                                            MODULES_NAV_STATUS.type,
                                            callback=self._status_cb)

        # Command services
        self._srv_cmd_fn = rospy.ServiceProxy(MODULES_NAV_SRV_CMD.name,
                                              MODULES_NAV_SRV_CMD.type)

        try:
            self._srv_cmd_fn.wait_for_service(timeout)
            self.loginfo('{} connected!'.format(MODULES_NAV_SRV_CMD.name))
        except rospy.ROSException:
            self.logerr('Timeout for waiting {}'.format(MODULES_NAV_SRV_CMD.name))
            return False
        except rospy.ROSInterruptException:
            self.logerr('Navigation controller shutting down...')
            return False

        # Action server
        self.act_cli = SimpleActionClient(MODULES_NAV_ACT_NAV.name,
                                            MODULES_NAV_ACT_NAV.type)

        if not self.act_cli.wait_for_server(timeout=rospy.Duration(timeout)):
            self.logerr('Wait {} timeout!'.format(MODULES_NAV_ACT_NAV.name))
            return False
        self.loginfo('{} connected!'.format(MODULES_NAV_ACT_NAV.name))

        self.logwarn('Nav server connected!!')
        self.is_connected = True
        return True

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        '''
        Sending the goal to server

        Parameters
        ----------
        goal : boothbot_msgs.msg.NavigationGoal
            The goal

        done_cb: callable
            done_cb Callback that gets called on transitions to
            Done. The callback should take two parameters: the terminal
            state (as an integer from actionlib_msgs/GoalStatus) and the
            result.

        active_cb: callable
            No-parameter callback that gets called on transitions to Active.

        feedback_cb: callable
            Callback that gets called whenever feedback

        Returns
        -------
        bool
            If the goal sent or not
        '''
        # sending goal...
        self.logwarn('Sending goal: {}'.format(goal))
        self.act_cli.send_goal(goal,
                                   done_cb=done_cb,
                                   active_cb=active_cb,
                                   feedback_cb=feedback_cb)
        return True

    def cancel_goal(self):
        '''
        Cancel the last goal from this client
        '''
        self.act_cli.cancel_goal()
        return True

    def reset(self):
        '''
        Set `RESET` command to server
        '''
        return self._srv_cmd_fn(CS.RESET.name)

    def set_command(self, cmd):
        '''
        Sending the command to server

        Parameters
        ----------
        cmd : boothbot_nav.states.NavigationServerCommand
            The cmd

        Returns
        -------
        bool
            If the cmd was received or not
        '''
        # legal check
        try:
            if not isinstance(cmd, CS):
                cmd = CS[cmd.upper()]
            return self._set_srv_cmd(cmd.name)
        except KeyError:
            self.logwarn('No such command: {}'.format(cmd))
            return False

    #######################################################################
    #  Internal
    #######################################################################
    def _status_cb(self, msg):
        self._msg_status = msg

    def _set_srv_cmd(self, cmd):
        # online check
        if not self.is_connected:
            return False

        try:
            res = self._srv_cmd_fn(cmd)
            return res.accepted

        except rospy.service.ServiceException as e:
            self.logwarn('service is unavailable')
            self.logerr(e)
            return False


################################################################################
# Test run
################################################################################
import boothbot_msgs.msg as bbmsgs
import geometry_msgs.msg as gemsgs

from boothbot_common.utils import xyrz_from_msg_pose, shift_point
from boothbot_msgs.ros_interfaces import (
    DEBUG_RVIZ_SIMPLE_GOAL,
    DEBUG_NAV_4D,
    DEBUG_NAV_4D_RESULT,
    DEBUG_MANUAL_COMMAND,
    DEBUG_NAV_DISABLE_LAYERS,
    MODULES_CB_PDO
)
from boothbot_nav.settings import MARK_OFFSET, DEBUG_4D_TOLERANCE, DEBUG_SIMPLE_GOAL_TOLERANCE

from dynamic_reconfigure.client import Client as DynamicReconfigClient

MOVE_BASE_NODE = "/move_base"
GLOBAL_COSTMAP = "/global_costmap"
LOCAL_COSTMAP = "/local_costmap"

OBSTACLE_LAYERS = [
    "/inflation_layer",
    "/pointcloud_layer",
    "/global_rgbd_obstacle_layer",
    "/lidar_obstacle_layer",
    "/sonar_obstacle_layer",
]

def get_obstacle_layers_reconfig_clients(costmap_list, obstacle_layer_list):
    clients = []
    for costmap in costmap_list:
        for layer in obstacle_layer_list:
            try:
                reconfig_node_name = MOVE_BASE_NODE + costmap + layer
                client = DynamicReconfigClient(reconfig_node_name, timeout=0.5)
                clients.append(client)
                logger.loginfo("{} connected!".format(client.name))
            except rospy.ROSException as e:
                logger.logwarn("Cannot connect to: {}".format(reconfig_node_name))
    return clients


def disable_layers(clients):
    for client in clients:
        client.update_configuration({"enabled": False})
        logger.loginfo("{} disabled!".format(client.name))
    return True

def enable_layers(clients):
    for client in clients:
        client.update_configuration({"enabled": True})
        logger.loginfo("{} enabled!".format(client.name))
    return True

if __name__ == '__main__':
    rospy.init_node('navigation_module_test_client', log_level=rospy.DEBUG)
    module = NavigationClientROS(parent_prefix='debug')
    module.connect()

    mark_offset_buffered = MARK_OFFSET
    layer_clients = get_obstacle_layers_reconfig_clients((GLOBAL_COSTMAP, LOCAL_COSTMAP), OBSTACLE_LAYERS)
    pub_4d_result = DEBUG_NAV_4D_RESULT.Publisher()
    msg_gs_hub_pdo = MODULES_GS_HUB_PDO.type()

    def _gs_hub_pdo_cb(msg):
        global msg_gs_hub_pdo
        msg_gs_hub_pdo = msg

    MODULES_GS_HUB_PDO.Subscriber(callback=_gs_hub_pdo_cb)

    def debug_done_cb(state, result):
        global layer_clients, pub_4d_result
        module.logwarn('Done with {} and {}'.format(state, result))
        logger.logwarn("Enabling obstacle layers...")
        logger.logwarn("Publishing result...")
        pub_4d_result.publish(result)
        enable_layers(layer_clients)

    def debug_rviz_simple_goal_cb(msg):
        '''
        This func recived the `goal_2d` from rviz, it generate goal to to control our nav server
        '''
        goal = bbmsgs.NavigationGoal()
        goal.stamp = msg.header.stamp
        goal.target_pose = xyrz_from_msg_pose(msg.pose)
        goal.goal_allowance = DEBUG_SIMPLE_GOAL_TOLERANCE
        global msg_gs_hub_pdo
        for gs in msg_gs_hub_pdo.gs_info:
            if gs.seq:
                goal.valid_gs.append(int(gs.serial_code.split("-")[-1]))
                break
        if not goal.valid_gs:
            logger.logerr("No valid GS!")
            return
        goal.last_moving_factor = 1.
        module.send_goal(goal, done_cb=debug_done_cb)

    def debug_nav_4d_cb(msg):
        '''
        This func reciving (float, float, float) from `nav_4d` for the 4 direction test only, it offsets the goal by
        mark_offset_buffered, which could be modified by `manual_command`.
        '''
        global mark_offset_buffered, layer_clients
        logger.logwarn("Receving debug_4d_goal, disabling obstacle layers...")
        disable_layers(layer_clients)
        goal = bbmsgs.NavigationGoal()
        goal.stamp = rospy.Time.now()
        xyrz = msg.data[:3]
        # shift the point for marking at the offset place
        goal.target_pose = list(shift_point(xyrz[:2], xyrz[2], -mark_offset_buffered)) + [xyrz[2]]
        goal.goal_allowance = DEBUG_4D_TOLERANCE
        # TODO: should we update the teb tolerance also? it is by default: (0.002, 0.002, 0.002)
        global msg_gs_hub_pdo
        for gs in msg_gs_hub_pdo.gs_info:
            if gs.seq:
                goal.valid_gs.append(int(gs.serial_code.split("-")[-1]))
                break
        if not goal.valid_gs:
            logger.logerr("No valid GS!")
            return
        goal.last_moving_factor = 1.
        module.logwarn('Goal with mark offset: {}'.format(mark_offset_buffered))
        module.send_goal(goal, done_cb=debug_done_cb)

    def debug_manual_command_cb(msg):
        '''
        `manual_command` callback for update mark_offset_buffered here only.
        '''
        global mark_offset_buffered
        cmd_l = msg.data.split(':')
        if cmd_l[0] == 'MARK_OFFSET':
            try:
                mark_offset_buffered = float(cmd_l[1])
                module.logwarn('Current mark offset is updated to: {}'.format(mark_offset_buffered))
            except ValueError:
                pass

    def debug_disable_layers_cb(msg):
        global layer_clients
        if msg.data == True:
            disable_layers(layer_clients)
        else:
            enable_layers(layer_clients)

    DEBUG_RVIZ_SIMPLE_GOAL.Subscriber(callback=debug_rviz_simple_goal_cb)
    DEBUG_NAV_4D.Subscriber(callback=debug_nav_4d_cb)
    DEBUG_MANUAL_COMMAND.Subscriber(callback=debug_manual_command_cb)
    DEBUG_NAV_DISABLE_LAYERS.Subscriber(callback=debug_disable_layers_cb)
    rospy.spin()
