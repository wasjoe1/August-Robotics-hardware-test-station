#!/usr/bin/python3

from timeit import default_timer as timer
from enum import Enum, auto
from pprint import PrettyPrinter

import time
import sys

import logging
from transitions import Machine
logging.getLogger("transitions").setLevel(logging.WARN)

class TaskDispatcher():
    '''
    Remarks: currently not thread safe, 
    maybe unstable in ros (thread-based callback in python)
    '''

    class State(Enum):
        INIT = 0
        # issue(cmd)->wait(4feedback)->exec
        ISSUE  = auto()
        WAIT = auto()
        EXEC = auto()
        # exec->(2)outcomes: success/fail
        SUCCEED = auto()
        FAILED = auto()
        # intermidiate states: can still escape by ext calls
        PAUSED = auto()
        # terminal states: no escape from these states
        SKIPPED = auto()
        COMPLETED = auto()
        ABORTED = auto()

    TASK_DISPATCH_AUTO_TRANS = [
        # Source: State.INIT
        {
            'trigger': '_poll',
            'source': State.INIT,
            'dest': State.ISSUE,
            'conditions': ['is_auto_start','is_valid'],
            'before':'clean_cmd_input',
        },
        {
            'trigger': '_poll',
            'source': State.INIT,
            'dest': State.SKIPPED,
            'conditions': ['is_auto_start','is_not_valid'],
            'after': ['log_invalid_task', 'process_cb'],
        },
        {
            'trigger': '_poll',
            'source': State.INIT,
            'dest': None,
        },
        # Source: State.ISSUE
        {
            'trigger':'_poll',
            'source': State.ISSUE,
            'dest': State.WAIT,
            'conditions': 'is_ready',
            'before': ['issue_cmd', 'set_issue_timer'],
        },
        {
            'trigger':'_poll',
            'source': State.ISSUE,
            'dest': None, # loop-back: wait or `is_ready`
        },
        # Source: State.WAIT
        {
            'trigger':'_poll',
            'source': State.WAIT,
            'dest': State.EXEC,
            'conditions': 'is_cmd_accepted',
            'before':  'set_exec_timer',
        },
        {
            'trigger':'_poll',
            'source': State.WAIT,
            'dest': State.FAILED,
            'conditions': 'is_timeout',
            'after': ['log_timeout', 'unset_timer'],
        },
        {
            'trigger':'_poll',
            'source': State.WAIT,
            'dest': None,       # loop-back
        },
        # Source: State.EXEC
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': State.SUCCEED,
            'conditions': ['is_exec_no_fault','is_exec_done'],
        },
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': State.FAILED,
            'conditions': 'is_exec_faulted',
            'after': ['log_fault_task'],
        },
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': State.FAILED,
            'conditions': 'is_timeout',
            'after': ['log_timeout', 'unset_timer'],
        },
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': State.PAUSED,
            'conditions': 'is_exec_interrupted',
            'after': ['log_pause'],
        },
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': State.ABORTED,
            'conditions': 'is_exec_canceled',
            'after': ['log_abort'],
        },
        {
            'trigger':'_poll',
            'source': State.EXEC,
            'dest': None,       # loop-back
            'conditions': 'is_exec_no_fault',
            'after': ['no_op'],
        },
        # Source: State.PAUSED
        {
            'trigger':'_poll',
            'source': State.PAUSED,
            'dest': State.EXEC,
            'conditions': 'is_cmd_accepted',
            'after': ['log_resume'],
        },
        {
            'trigger':'_poll',
            'source': State.PAUSED,
            'dest': State.FAILED,
            'conditions': 'is_timeout',
            'after': ['log_timeout', 'unset_timer'],
        },
        {
            'trigger':'_poll',
            'source': State.PAUSED,
            'dest': State.ABORTED,
            'conditions': 'is_exec_canceled',
            'after': ['log_abort'],
        },
        # Source: State.FAILED
        {
            'trigger':'_poll',
            'source': State.FAILED,
            'dest': State.ISSUE,
            'unless': 'is_retry_limit_reached',
            'after': ['increment_retry_count', 'cancel_cmd', 'clean_cmd_input'],
        },
        {
            'trigger':'_poll',
            'source': State.FAILED,
            'dest': State.SKIPPED,
            'conditions': 'is_retry_limit_reached',
            'after': ['cancel_cmd', 'process_cb',], # (???)
        },
        # Source: State.SUCCEED
        {
            'trigger':'_poll',
            'source': State.SUCCEED,
            'dest': State.COMPLETED,
            'after': ['process_cb'],
        },
    ]

    UNRECOVERABLE_ERROR_TRANS = [
        # `_abort()` -> jump to aborted state, halt everything (no next hook)
        {
            'trigger':'_abort',
            'source': [State.SKIPPED,State.COMPLETED,State.ABORTED],
            'dest': None,   # explicitly declared to prevent recursion
        },
        {
            'trigger':'_abort',
            'source': '*',
            'dest': State.ABORTED,
            'after': ['log_abort', 'cancel_cmd', 'process_cb'],
        },
    ]

    # The following call is async: meaning it will execute without waiting
    # the next `_poll()` call.
    # Most of the calls are created in pairs, with acl-like "deny/allow" items,
    # as we would like `ignore_invalid_triggers` (in transitions) to be turned off
    # for readibilty and robustness.
    EXTERNAL_CALL_TRANS = [

        # `_start()` -> kickstart the sequnce
        {
            'trigger': '_start',
            'source': State.INIT,
            'dest': State.ISSUE,
            'conditions': 'is_valid',
            'before': ['clean_cmd_input'],
        },
        {
            'trigger': '_start',
            'source': State.INIT,
            'dest': State.SKIPPED,
            'unless': 'is_valid',
            'after': ['log_invalid_task', 'process_cb'],
        },
        {
            'trigger': '_start',
            'source': '*',
            'dest': None,
            # no-op when calling start on already started tasks
        },
        # `_pause()` -> save current state and stops
        {
            'trigger':'_pause',
            'source': '*',
            'dest': None,
            'conditions': ['is_task_ended'],
            # forbid terminal states from changing
        },
        {
            'trigger':'_pause',
            'source': '*',
            'dest': State.PAUSED,
            'unless': 'is_task_ended',
            'after': ['log_pause', 'pause_cmd'],
        },
        # `_resume()` -> resume from saved state
        {
            'trigger':'_resume',
            'source': State.PAUSED,
            'dest': State.WAIT,
            'after': ['log_resume', 'resume_cmd'],
        },
        {
            'trigger':'_resume',
            'source': '*',
            'dest': None,
            # no-op for non-paused state
        },
        # `_skip()` -> force skipping over the task, exec next hook
        {
            'trigger':'_skip',
            'source': '*',
            'dest': None,
            'conditions': 'is_task_ended',
            # forbid terminal states from changing
        },
        {
            'trigger':'_skip',
            'source': '*',
            'dest': State.SKIPPED,
            'unless': 'is_task_ended',
            'after': ['log_interrupt', 'cancel_cmd', 'process_cb'], # (+??)
        },
    ]

    LOOPBACK_STATE_TRANS = [
        # Source: State.INIT
        {
            'trigger':'_poll',
            'source': State.INIT,
            'dest': None,       # loop-back: (pseudo-)terminal state
        },
        # Source: State.PAUSED
        {
            'trigger':'_poll',
            'source': State.PAUSED,
            'dest': None,       # loop-back: (pseudo-)terminal state
        },

        # Source: State.SKIPPED
        {
            'trigger':'_poll',
            'source': State.SKIPPED,
            'dest': None,       # loop-back: terminal state
        },
        # Source: State.COMPLETED
        {
            'trigger':'_poll',
            'source': State.COMPLETED,
            'dest': None,       # loop-back: terminal state
        },
        # Source: State.ABORTED
        {
            'trigger':'_poll',
            'source': State.ABORTED,
            'dest': None,       # loop-back: terminal state
        },
    ]

    transitions = UNRECOVERABLE_ERROR_TRANS + EXTERNAL_CALL_TRANS +\
                    TASK_DISPATCH_AUTO_TRANS + LOOPBACK_STATE_TRANS

    class Report():
        # drop a reference here:
        # https://stackoverflow.com/questions/472000/usage-of-slots
        __slots__=[
            'start_time',
            'end_time',
            'retry_count',
            'max_retry',
            'paused',
            'history',
            'error_state',
            'error_reason',
            # 'last_mod_time',
        ]

        def __init__(self):
            self.start_time = None
            self.end_time = None
            self.retry_count = 0
            self.max_retry = 0
            self.paused = False
            self.error_state = None
            self.error_reason = []
            self.history = []

        @property
        def started(self):
            return self.start_time != None

        @property
        def done(self):
            return self.end_time != None

        def log_state(self, trans):
            '''adding the current state to the history'''
            self.history.append(f"{trans.source} -> {trans.dest}")

        def json(self):
            return {key : getattr(self, key, None) for key in self.__slots__}

    class InvalidCallbackException(Exception):
        pass
    
    class UnsuccessfulCallbackException(Exception):
        pass

    def __init__(self, auto_start=False, max_retry=0, issue_timeout=None, exec_timeout=None,
        on_complete=None, on_skip=None, on_abort=None, verbose=False):

        self.timeout_target = None
        self.timeouts = {
            'issue': issue_timeout,
            'exec': exec_timeout,
        }

        self.auto_start = auto_start

        self.verbose = verbose
        self.report = TaskDispatcher.Report()
        self.report.max_retry = max_retry

        self.state_hooks = {
            # on terminal states
            'SKIPPED':  on_skip,
            'COMPLETED':  on_complete,
            'ABORTED':  on_abort,
        }

        self.machine = Machine(
            model=self,
            states=TaskDispatcher.State,
            initial=TaskDispatcher.State.INIT,
            transitions=TaskDispatcher.transitions,
            after_state_change='log_state_change',
            ignore_invalid_triggers=False,
            on_exception='handle_error',            # need transitions version 0.8.9 
            auto_transitions=False,                 # no 'to_<state>' hooks
            send_event=True,                        # use 'EventData' wrapping
        )

    def __del__(self):
        if self.verbose:
            self.print_report()


    ### Conditions: `@property` decorator

    @property
    def is_valid(self) -> bool:
        raise NotImplementedError
    
    def is_not_valid(self) -> bool:
        return not self.is_valid

    @property
    def is_ready(self) -> bool:
        raise NotImplementedError

    @property
    def is_cmd_accepted(self) -> bool:
        raise NotImplementedError

    @property
    def is_timeout(self) -> bool:
        # timestamp=None implies no active timer set
        if not self.timeout_target:
            return False
        # target=None implies timeout is infinite
        elif not self.timeout_target:
            return False
        else:
            current_time = timer()
            return bool(current_time>=self.timeout_target)

    @property
    def is_exec_faulted(self) -> bool:
        raise NotImplementedError

    @property
    def is_exec_no_fault(self) -> bool:
        return not self.is_exec_faulted

    @property
    def is_exec_done(self) -> bool:
        raise NotImplementedError

    @property
    def is_exec_interrupted(self) -> bool:
        raise NotImplementedError

    @property
    def is_exec_canceled(self) -> bool:
        raise NotImplementedError

    @property
    def is_retry_limit_reached(self) -> bool:
        return self.report.retry_count >= self.report.max_retry

    @property
    def is_auto_start(self) -> bool:
        return self.auto_start

    @property
    def is_task_ended(self) -> bool:
        return self.state in [TaskDispatcher.State.SKIPPED,
            TaskDispatcher.State.COMPLETED, TaskDispatcher.State.ABORTED]
    
    @property
    def is_task_successful(self) -> bool:
        return self.state == TaskDispatcher.State.COMPLETED

    @property
    def is_task_unrecoverable(self) -> bool:
        return self.state == TaskDispatcher.State.ABORTED

    ### Actions/hooks
    def clean_cmd_input(self, event):
        raise NotImplementedError

    def issue_cmd(self, event):
        raise NotImplementedError

    def cancel_cmd(self, event):
        raise NotImplementedError

    def pause_cmd(self, event):
        raise NotImplementedError

    def resume_cmd(self, event):
        raise NotImplementedError

    def process_cb(self, event):
        '''
        Processing callback/hooks attached at fsm construction,
        utilizing event.transition to deterime which hook to trigger

        Example event wrapping
        
        "event.__dict__"
        { 'args': (),
          'error': None,
          'event': <Event('_poll')@140542122263216>,
          'kwargs': {},
          'machine': <transitions.core.Machine object at 0x7fd2852072b0>,
          'model': <__main__.TaskDispatcherExperiment object at 0x7fd2852072e0>,
          'result': False,
          'state': <State('ISSUE')@140542153368768>,
          'transition': <Transition('ISSUE', 'WAIT')@140542122264080>}

        "event.transition.__dict__"
        { 'after': [],
          'before': ['issue_cmd', 'refresh_timeout'],
          'conditions': [<Condition(is_ready)@139633942551616>],
          'dest': 'WAIT',
          'prepare': [],
          'source': 'ISSUE'}
        
        Extract transition info from: `event.transition` -> `.source` & `.dest`
        '''

        if event.transition.dest in ['SKIPPED', 'COMPLETED', 'ABORTED']:
            # confirm that the callback is processed in the 'after' part
            # remarks: temp fix for type mismatch -> transitions.core.State
            if (str(event.state.name) != str(event.transition.dest)):
                raise TaskDispatcher.InvalidCallbackException
            if self.state_hooks[event.transition.dest] is not None:
                try:
                    self.state_hooks[event.transition.dest]()
                except Exception as e:
                    raise TaskDispatcher.UnsuccessfulCallbackException from e
        else:
            raise TaskDispatcher.InvalidCallbackException


    def set_issue_timer(self, event):
        if self.timeouts["issue"]:
            self.timeout_target = timer() + float(self.timeouts["issue"])

    def set_exec_timer(self, event):
        if self.timeouts["exec"]:
            self.timeout_target = timer() + float(self.timeouts["exec"])

    def unset_timer(self, event):
        self.timeout_target = None

    def increment_retry_count(self, event):
        self.report.retry_count += 1

    def no_op(self, event):
        pass

    ## Actions/internal log&report

    def handle_error(self, event):
        '''default handler: print traceback & fsm report'''
        stderr_pp = PrettyPrinter(indent=2, stream=sys.stderr)
        print("" ,file=sys.stderr)
        print("--- FSM event ---" ,file=sys.stderr)
        stderr_pp.pprint(event.__dict__)
        print("--- FSM report ---" ,file=sys.stderr)
        stderr_pp.pprint(self.report.json())
        print("--- end FSM traceback ---" ,file=sys.stderr)

    def print_report(self):
        print(f"--- FSM report {self} ---" ,file=sys.stderr)
        PrettyPrinter(indent=2, stream=sys.stderr).pprint(self.report.json())

    def log_state_change(self, event):
        t = event.transition
        if t.source != t.dest and t.dest is not None:
            self.report.log_state(t)
        if self.verbose:
            print("===", file=sys.stderr)
            PrettyPrinter(indent=2, stream=sys.stderr).pprint(event)
            print("===", file=sys.stderr)

    def log_invalid_task(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = event.transition.source
        self.report.error_reason += ["INVALID"]

    def log_timeout(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = event.transition.source
        self.report.error_reason += ["TIMEOUT"]
    
    def log_success_task(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = None
        self.report.error_reason = []

    def log_fault_task(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = event.transition.source
        self.report.error_reason += ["FAULT"]

    def log_abort(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = event.transition.source
        self.report.error_reason += ["ABORT"]

    def log_interrupt(self, event):
        self.report.end_time = int(time.time())
        self.report.error_state = event.transition.source
        self.report.error_reason += ["INTERRUPTED"]

    def log_pause(self, event):
        self.report.paused = True
        self.report.error_state = event.transition.source
        self.report.error_reason += ["PAUSE"]

    def log_resume(self, event):
        self.report.paused = False
        self.report.error_state = None
        self.report.error_reason = []

# taking inspiration here:
# https://www.smashingmagazine.com/201m8/01/rise-state-machines/
# https://www.npmjs.com/package/@newstudios/advanced
# https://www.researchgate.net/figure/State-transition-diagram-of-the-process-scheduler-in-RTOS_fig2_220636928