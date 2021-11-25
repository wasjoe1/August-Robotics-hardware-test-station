#!/usr/bin/python3

from taskex import TaskDispatcher
import pytest
import time

class TaskDispatcherBase(TaskDispatcher):

    @property
    def is_valid(self) -> bool:
        return True

    @property
    def is_ready(self) -> bool:
        return True

    @property
    def is_cmd_accepted(self) -> bool:
        return True

    @property
    def is_exec_faulted(self) -> bool:
        return False

    @property
    def is_exec_done(self) -> bool:
        return True

    @property
    def is_exec_interrupted(self) -> bool:
        return False
    
    @property
    def is_exec_canceled(self) -> bool:
        return False

    ### Actions/hooks
    def clean_cmd_input(self, event):
        pass

    def issue_cmd(self, event):
        pass

    def cancel_cmd(self, event):
        pass

    def pause_cmd(self, event):
        pass

    def resume_cmd(self, event):
        pass

def test_init_state():
    td = TaskDispatcherBase()
    assert td.state == TaskDispatcher.State.INIT
    td._poll()
    assert td.state == TaskDispatcher.State.INIT

def test_init_auto_start():
    td = TaskDispatcherBase(auto_start=True)
    assert td.state == TaskDispatcher.State.INIT
    td._poll()
    assert td.state == TaskDispatcher.State.ISSUE

def test_implemented():
    td = TaskDispatcherBase()
    assert td.is_valid == True
    assert td.is_ready == True
    assert td.is_cmd_accepted == True
    assert td.is_exec_no_fault == True 
    assert td.is_exec_done == True

    assert td.is_exec_faulted == False
    assert td.is_exec_interrupted == False
    assert td.is_timeout == False
    assert td.is_retry_limit_reached is not None

    try:
        td.clean_cmd_input(None)
        td.issue_cmd(None)
        td.cancel_cmd(None)
    except Exception as e:
        pytest.fail(f"Unexpected Exception {e}")

def test_complete_no_error():
    td = TaskDispatcherBase()
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.SUCCEED
    td._poll()
    assert td.state == TaskDispatcher.State.COMPLETED
    td._poll()
    assert td.state == TaskDispatcher.State.COMPLETED
    hist = td.report.history
    assert len(hist) == 5   # 6 states visited, 5 transitions

def test_repeated_start():
    td = TaskDispatcherBase()
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.SUCCEED
    td._poll()
    assert td.state == TaskDispatcher.State.COMPLETED
    td._poll()
    assert td.state == TaskDispatcher.State.COMPLETED


def test_skip():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if td.is_task_ended:
            continue
        td._skip()
        assert td.state == TaskDispatcher.State.SKIPPED

def test_skip_ended():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if not td.is_task_ended:
            continue
        td._skip()
        assert td.state == s


def test_abort():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if td.is_task_ended:
            continue
        td._abort()
        assert td.state == TaskDispatcher.State.ABORTED

def test_abort_ended():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if not td.is_task_ended:
            continue
        td._abort()
        assert td.state == s


def test_resume():
    td = TaskDispatcherBase()
    td.state = TaskDispatcher.State.PAUSED
    td._resume()
    assert td.state == TaskDispatcher.State.WAIT

def test_resume_non_paused():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        if s == TaskDispatcher.State.PAUSED:
            continue
        td.state = s
        td._resume()
        assert td.state == s


def test_pause():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if td.is_task_ended:
            continue
        td._pause()
        assert td.state == TaskDispatcher.State.PAUSED

def test_pause_ended():
    td = TaskDispatcherBase()
    for s in TaskDispatcher.State:
        td.state = s
        if not td.is_task_ended:
            continue
        td._pause()
        assert td.state == s

class TaskDispatcherTaskPausedDetect(TaskDispatcherBase):

    @property
    def is_exec_done(self) -> bool:
        return False

    @property
    def is_exec_interrupted(self) -> bool:
        return True

def test_pause_detection():
    td = TaskDispatcherTaskPausedDetect()
    td.state = TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.PAUSED

class TaskDispatcherTaskResumeDetect(TaskDispatcherBase):

    @property
    def is_exec_done(self) -> bool:
        return False

    @property
    def is_cmd_accepted(self) -> bool:
        return True

def test_resume_detection():
    td = TaskDispatcherTaskResumeDetect()
    td.state = TaskDispatcher.State.PAUSED
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC


class TaskDispatcherTaskCanceledDetect(TaskDispatcherBase):

    @property
    def is_cmd_accepted(self) -> bool:
        return False

    @property
    def is_exec_done(self) -> bool:
        return False

    @property
    def is_exec_canceled(self) -> bool:
        return True

def test_exec_cancel_detection():
    td = TaskDispatcherTaskCanceledDetect()
    td.state = TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.ABORTED

def test_paused_cancel_detection():
    td = TaskDispatcherTaskCanceledDetect()
    td.state = TaskDispatcher.State.PAUSED
    td._poll()
    assert td.state == TaskDispatcher.State.ABORTED


class TaskDispatcherIssueBusy(TaskDispatcherBase):

    @property
    def is_cmd_accepted(self) -> bool:
        return False

def test_issue_timeout():
    td = TaskDispatcherIssueBusy(max_retry=0, issue_timeout=.1, exec_timeout=.1)
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    time.sleep(0.2)
    td._poll()
    assert td.state == TaskDispatcher.State.FAILED
    td._poll()
    assert td.state == TaskDispatcher.State.SKIPPED


class TaskDispatcherExecBusy(TaskDispatcherBase):

    @property
    def is_exec_done(self) -> bool:
        return False

def test_exec_timeout():
    td = TaskDispatcherExecBusy(max_retry=0, issue_timeout=.1, exec_timeout=.1)
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    time.sleep(0.2)
    td._poll()
    assert td.state == TaskDispatcher.State.FAILED
    td._poll()
    assert td.state == TaskDispatcher.State.SKIPPED


class TaskDispatcherPausedBusy(TaskDispatcherBase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cmd_accepted = True

    @property
    def is_exec_done(self) -> bool:
        return False

    @property
    def is_cmd_accepted(self) -> bool:
        return self.cmd_accepted

def test_paused_timeout():
    td = TaskDispatcherPausedBusy(max_retry=0, issue_timeout=.1, exec_timeout=.1)
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    td.cmd_accepted = False
    assert td.is_cmd_accepted == False
    td._pause()
    assert td.state == TaskDispatcher.State.PAUSED
    td._poll()
    assert td.state == TaskDispatcher.State.PAUSED
    time.sleep(0.2)
    td._poll()
    assert td.state == TaskDispatcher.State.FAILED
    td._poll()
    assert td.state == TaskDispatcher.State.SKIPPED


class TaskDispatcherExecFail(TaskDispatcherBase):

    @property
    def is_exec_faulted(self) -> bool:
        return True


def test_retry_once():
    td = TaskDispatcherExecFail(max_retry=1, issue_timeout=.1, exec_timeout=.1)
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    assert td.state == TaskDispatcher.State.ISSUE
    td._poll()
    assert td.state == TaskDispatcher.State.WAIT
    td._poll()
    assert td.state == TaskDispatcher.State.EXEC
    td._poll()
    assert td.state == TaskDispatcher.State.FAILED
    td._poll()
    assert td.state == TaskDispatcher.State.ISSUE

def test_retry_twice():
    td = TaskDispatcherExecFail(max_retry=2, issue_timeout=.1, exec_timeout=.1)
    assert td.state == TaskDispatcher.State.INIT
    td._start()
    for i in range(2+1):
        assert td.state == TaskDispatcher.State.ISSUE
        td._poll()
        assert td.state == TaskDispatcher.State.WAIT
        td._poll()
        assert td.state == TaskDispatcher.State.EXEC
        td._poll()
        assert td.state == TaskDispatcher.State.FAILED
        td._poll()
    td._poll()
    assert td.state == TaskDispatcher.State.SKIPPED
    

class TaskDispatcherHandleException(TaskDispatcherBase):

    @property
    def is_valid(self) -> bool:
        raise Exception

    def handle_error(self, event):
        super().handle_error(event)
        del event.error
        self._abort()


def test_exception_abort():
    td = TaskDispatcherHandleException()
    assert td.state == TaskDispatcher.State.INIT
    td._start() # exception raised & suppressed by handler
    assert td.state == TaskDispatcher.State.ABORTED


class DummyHook():
    def __init__(self):
        self.called = False

    def hook(self):
        self.called = True

class TaskDispatcherCallback(TaskDispatcherBase):
    pass

def test_on_complete_cb():
    target = DummyHook()
    td = TaskDispatcherCallback(on_complete=target.hook)
    td.state = TaskDispatcher.State.SUCCEED
    td._poll()
    assert td.state == TaskDispatcher.State.COMPLETED
    assert target.called == True

def test_on_skip_cb():
    target = DummyHook()
    td = TaskDispatcherCallback(on_skip=target.hook)
    td.state = TaskDispatcher.State.SUCCEED
    td._skip()
    assert td.state == TaskDispatcher.State.SKIPPED
    assert target.called == True

def test_on_abort_cb():
    target = DummyHook()
    td = TaskDispatcherCallback(on_abort=target.hook)
    td._abort()
    assert td.state == TaskDispatcher.State.ABORTED
    assert target.called == True

### Testing if all the required transition methods are implemented,
### This need to be tested for each subclass

def get_all_triggers(fsm):
    trigger_list = [ t['trigger'] for t in fsm.transitions ] 
    return set(trigger_list)

def get_all_conditions(fsm):
    cond_list = []
    for t in fsm.transitions:
        if 'conditions' in t:
            if isinstance(t['conditions'], list):
                cond_list.extend(t['conditions'])
            elif isinstance(t['conditions'], str):
                cond_list.append(t['conditions'])
        
        if 'unless' in t:
            if isinstance(t['unless'], list):
                cond_list.extend(t['unless'])
            elif isinstance(t['unless'], str):
                cond_list.append(t['unless'])
    return set(cond_list)

def get_all_actions(fsm):
    action_list = []
    for t in fsm.transitions:
        if 'before' in t:
            if isinstance(t['before'], list):
                action_list.extend(t['before'])
            elif isinstance(t['before'], str):
                action_list.append(t['before'])

        if 'after' in t:
            if isinstance(t['after'], list):
                action_list.extend(t['after'])
            elif isinstance(t['after'], str):
                action_list.append(t['after'])
    return set(action_list)

def test_validate_fsm_triggers():
    td = TaskDispatcherBase()
    for trig in get_all_triggers(td):
        for s in TaskDispatcher.State:
            td.state = s
            getattr(td, trig)()

def test_validate_fsm_conditions():
    td = TaskDispatcherBase()
    for cond in get_all_conditions(td):
        assert getattr(td, cond) is not None

class DummyObject():
    pass

class DummyEvent():
    def __init__(self):
        self.args = ()
        self.kargs = {}
        self.state = None
        self.transition = DummyObject()
        self.transition.source = None
        self.transition.dest = None

def test_validate_fsm_actions():
    td = TaskDispatcherBase()
    # Need to tweak this dummy event if the usage chages
    for act in get_all_actions(td):
        try:
            getattr(td, act)(DummyEvent())
        except TaskDispatcher.InvalidCallbackException:
            pass

def get_not_implemented_conditions(fsm):
    cond_set = get_all_conditions(fsm)
    error_list = []
    for cond in cond_set:
        try:
            getattr(fsm, cond)
        except NotImplementedError:
            error_list.append(cond)
    return set(error_list)

def get_not_implemented_actions(fsm):
    act_set = get_all_actions(fsm)
    error_list = []
    for act in act_set:
        try:
            getattr(fsm, act)(DummyEvent())
        except TaskDispatcher.InvalidCallbackException:
            pass
        except NotImplementedError:
            error_list.append(act)
    return set(error_list)

def test_not_implemented():
    td = TaskDispatcher()
    assert len(get_not_implemented_conditions(td)) != 0
    assert len(get_not_implemented_actions(td)) != 0
    td = TaskDispatcherBase()
    assert len(get_not_implemented_conditions(td)) == 0
    assert len(get_not_implemented_actions(td)) == 0


if __name__ == "__main__":
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2, compact=True)
    td = TaskDispatcher()
    pp.pprint("--- fsm triggers ---")
    pp.pprint(get_all_triggers(td))
    pp.pprint("--- fsm conditions ---")
    pp.pprint(get_all_conditions(td))
    pp.pprint("--- fsm actions ---")
    pp.pprint(get_all_actions(td))

    cond = get_not_implemented_conditions(td)
    act = get_not_implemented_actions(td)
    if cond or act:
        pp.pprint("--- not implemented ---")
        if cond:
            pp.pprint(cond)
        if act:
            pp.pprint(act)