#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from common.ros_topics import GUI_STATE_fn


class SimGSStatesSubs(object):
    """
    Subscribe all GSs and record their states.
    """

    def __init__(self, gs_seqs=[]):

        self.states = {}

        for gs_seq in gs_seqs:
            rospy.Subscriber(GUI_STATE_fn(gs_seq), String, self._state_cb_wrapper(gs_seq))
            self.states[gs_seq] = ""

    def _state_cb_wrapper(self, gs_seq):
        """ Use closure to keep self and gs_seq. """
        def _state_cb(msg):
            self.states[gs_seq] = msg.data.upper()

        return _state_cb
