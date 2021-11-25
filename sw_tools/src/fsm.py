#!/usr/bin/python3

from transitions import Machine
from timeit import default_timer as timer
from enum import Enum, auto
import random
import threading

import rospy
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import Trigger, TriggerResponse

'''
Similating a electronic oscillator, output a 1-hz square (or sawtooth) waveform,
with sync/aysnc reset, and state change counter.
The default output print the timedalta and fsm state to stdout

Subscribed Topics
---
None


Published Topics
---
~/state     (std_msg/String)
    internal state of the fsm
~/count     (std_msg/Int32)
    state change count since last reset
~/output    (std_msg/Bool)
    output of the resultant waveform


Services
---
~/sync_reset    (std_srvs/Trigger)
    reset the fsm, stage change counter, the output after reset is LOW
    (synchronous: wait until the next "clock edge")
~/async_reset   (std_srvs/Trigger)
    reset variables as in "~/sync_reset"
    (asynchronous: happen immediately, regardless of the "clock edge")

Services Called
---
None


Parameters
---
None
'''

class OscillatorFsm():

    class State(Enum):
        INIT = 0
        LOW = auto()    # LOW = 1
        HIGH = auto()   # HIGH = 2

    transitions = [
        {   
            # notice this will have the highest priority
            # in this way, reset is synchronized to the clock edge (trigger)
            'trigger':'_poll',
            'source': '*',
            'dest':State.INIT,
            'conditions':['is_reset',],
            'after':['reset_count', 'clear_reset', 'output_low'],
        },{   
            # another way to reset, this is explicitly defined as 'auto_transitions=False'
            # notice that reset is NOT synchronized to the clock edge (trigger)
            'trigger':'to_init',
            'source': '*',
            'dest':State.INIT,
            'after':['reset_count', 'clear_reset', 'output_low'],
        },{
            'trigger':'_poll',
            'source': State.INIT,
            'dest':State.HIGH,
            'prepare':['output_high'],
            'conditions':['unconditional_true',],
            # all condition = True in order to proceed
        },{
            'trigger':'_poll',
            'source': State.LOW,
            'dest':State.HIGH,
            'before':['increment_counter', 'output_high', "output_counter",],
        },{
            'trigger':'_poll',
            'source': State.HIGH,
            'dest':State.LOW,
            'before':['increment_counter', 'output_low', "output_counter",],
            # actually the difference is not significant, as nobody is using the state variable
            # https://github.com/pytransitions/transitions#callback-execution-order 
        },
    ]

    def __init__(self):
        self.machine = Machine(
            model=self,             # will create self._poll, self.state, etc.
            states=OscillatorFsm.State,
            initial=OscillatorFsm.State.INIT,
            transitions=OscillatorFsm.transitions,
            after_state_change='log_state_change',
            auto_transitions=False,                 # no 'to_<state>' hooks
            send_event=False,                       # no 'EventData' wrapping
        )
        self.state_pub = rospy.Publisher("~/state", String, queue_size=1)
        self.count_pub = rospy.Publisher("~/count", Int32, queue_size=1)
        self.output_pub = rospy.Publisher("~/output", Bool, queue_size=1)

        # term borrowed from digital circuits
        # ref: https://en.wikipedia.org/wiki/Asynchronous_circuit#Synchronous_vs_asynchronous_logic
        rospy.Service('~/sync_reset', Trigger, self.sync_reset)
        rospy.Service('~/async_reset', Trigger, self.async_reset)

        self._lock = threading.Lock()
        self.reset = False
        self.count = 0
        self.prev_time = None


    # conditions
    @property
    def unconditional_false(self):
        return False
    
    @property
    def unconditional_true(self):
        return True
    
    @property
    def is_reset(self):
       return self.reset

    @property
    def current_state(self):
        return str(self.state)
    
    @property
    def current_count(self):
        return int(self.count)


    # hooks
    def output_low(self):
        self.output_pub.publish(False)

    def output_high(self):
        self.output_pub.publish(True)

    def output_counter(self):
        self.count_pub.publish(self.current_count)

    def log_state_change(self):
        rospy.loginfo(f"Changing to {self.state}")
        now =  timer()
        if self.prev_time:
            rospy.loginfo(f"Time delta: {now-self.prev_time:.4f}")
        else:
            rospy.loginfo(f"Time delta: **start counting**")
        self.prev_time = now
        
    def increment_counter(self):
        with self._lock:
            self.count += 1

    def reset_count(self):
        with self._lock:
            self.count = 0
            
    def sync_reset(self, req):
        '''Flag the reset var and wait for the next poll'''
        with self._lock:
            self.reset = True
        return TriggerResponse(True, '')
    
    def async_reset(self, req):
        '''Jump straight back to init/low state'''
        self.to_init()
        return TriggerResponse(True, '')

    def clear_reset(self):
        with self._lock:
            self.reset = False

    def run_once(self):
        self._poll()
        fsm.publish_state()

    def publish_state(self):
        self.state_pub.publish(self.current_state)


if __name__ == "__main__":

    rospy.init_node('pytrans', anonymous=True)

    fsm = OscillatorFsm()
    rate = rospy.Rate(1*2)  # for a 1-hz oscillation frequency,
                            # or, in other words 2-hz control-loop cycle

    while not rospy.is_shutdown():
        fsm.run_once()  # equiv. to _poll(), but also execute sth extra
        rate.sleep()    # note: if the exec time > period, sleep is skipped


# __slots__ + subclassing -> ensure the required variables are declared
# before/after -> exception handling: fsm will stay at prev/next state