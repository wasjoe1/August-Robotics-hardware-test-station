
import rospy
from rostopic_helpers import BufferedTopicSubscriber

from threading import Lock
from collections import deque

# try:
#     from boothbot_common.errcode import ErrCode
# except ModuleNotFoundError:
#     pass
from errcode import ErrCode as ErrorCode
from boothbot_msgs.msg import ErrCode

class BufferedErrorCodeSubscriber(BufferedTopicSubscriber):

    def __init__(self, topic_name="/errcode", buf_size=1):
        self.info = (topic_name, ErrCode)
        self.buf_lock = Lock()
        self.buffer = {}
        self.sub = rospy.Subscriber(topic_name, ErrCode, self.callback)

    def callback(self, msg):
        with self.buf_lock:
            self.buffer[msg.header.frame_id] = msg.code

    def get_ec(self):
        with self.buf_lock:
            return [ v for (k,v) in self.buffer.items() if v>0 ]
            
    def get_readable_ec(self):
        return [ ErrorCode(ec).name for ec in self.get_ec() ]