import os
import time
import datetime
import json
from collections import OrderedDict

from common.errcode import ErrCode
from common.redis_keys import ERRCODE


class DashboardStates(object):

    # List special frame_id and its state redis key.
    # They will show state and error on GUI instead of error code only
    show_state_lst = []

    def __init__(self, R):
        assert len(self.show_state_lst) > 0, "No key list"
        self.R = R

    def _get_error_states(self):
        error_states_json = self.R.get(ERRCODE)
        error_states = {}
        if error_states_json is not None:
            error_states = json.loads(error_states_json)
        # Order error_states such that we can show error block
        # with better layout on GUI
        error_states = sorted(error_states.items(), key=lambda x: x[0])
        error_states = OrderedDict(error_states)
        return error_states

    def get(self):
        error_states = self._get_error_states()

        data = []
        for (frame_id, key) in self.show_state_lst:
            s = self.R.get(key)
            err_code = ErrCode.OK.value
            err_msg = ''
            if frame_id in error_states:
                err_code = error_states[frame_id]['err_code']
                err_msg = error_states[frame_id]['err_msg']
                # remove frame id to make next loop simpler
                error_states.pop(frame_id)
            data.append({
                'name': frame_id,
                # `show_state` indicate whether it's a special frame which need to show state of it
                'show_state': True,
                'state': s or 'No State',
                'err_code': err_code,
                'err_msg': err_msg,
            })
        for frame_id, err in error_states.iteritems():
            data.append({
                'name': frame_id,
                'show_state': False,
                'state': '',
                'err_code': err['err_code'],
                'err_msg': err['err_msg'],
            })

        return data

    def format(self, data_dict):
        """ Format each line of `get` results"""
        name = data_dict['name']
        state = data_dict['state']
        err_code = data_dict['err_code']
        err_msg = data_dict['err_msg']

        if err_code != ErrCode.OK.value:
            err_display = "ErrCode {}, {}".format(err_code, err_msg)
            # \033 for color display in terminal
            return "{}:\t \033[1;31;48m {} \033[1;37;48m \t{}.".format(name, state, err_display)

        return "{}:\t \033[1;32;48m {} \033[1;37;48m".format(name, state)

    def display(self):
        words = [self.format(d) for d in self.get()]
        if words:
            print '\n'.join(words)
        else:
            print 'No Data'

    def terminal_loop_display(self, sleep_time=5.0):
        # Looping forever
        assert sleep_time > 1, "Too short sleep"
        while True:
            os.system('clear')  # clear termial output
            print "Refresh at {}\n".format(datetime.datetime.now())
            self.display()
            time.sleep(sleep_time)
