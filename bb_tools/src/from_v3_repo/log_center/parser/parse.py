import re
import pandas as pd
from collections import OrderedDict

######
# Try to merge all log files and time sequential them
#####
def line_datetime(row):
    line = row['raw']
    # patter to find time
    p = r'.*\[(ros.*)\]\[(?P<log_level>(INFO|DEBUG|ERROR|WARNING))\] (?P<time>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):(?P<msg>.*)'
    r = re.match(p, line)
    if r:
        return pd.Series([r.group('time'), r.group('msg').strip(), r.group('log_level')])
    return pd.Series([None, None, None])


def read_csv(fpath):
    df = pd.read_csv(fpath, sep="###%%%", names=['raw'])  # we don't want to split row.
    df = df.apply(line_datetime, axis=1)
    df = df.dropna()
    df.rename(columns={0: 'time', 1: 'info', 2: 'log_level'}, inplace=True)
    return df


def read_csv_files(csv_lst):
    df = pd.concat([read_csv(fpath) for fpath in csv_lst], ignore_index=True)
    df['time'] = pd.to_datetime(df['time'], format='%Y-%m-%d %H:%M:%S,%f')
    df.set_index('time', inplace=True)
    df.sort_index(inplace=True)
    return df


######
# Try to pase goals. Related notebook is `Extract.ipynb`
######
class ParseGoal(object):

    single_goal_keys = (
        r'BBC: Received new goal with id (?P<id>\d+).*',
        lambda i: r'BBC: goal {} finished.*'.format(i)
    )

    internal_goal_key = r'gbm: Got new goal, internal id: (?P<internal_id>\d+).*'
    moving_keys = (
        r'nav: move_base actived.*',
        r'nav: State changed to LOCATING'
    )
    localization_keys = (
        lambda i: r'camera_beacon: Triggering goal: {} to ON.*'.format(i),
        r'nav: State changed to PENDING.*'
    )
    marking_keys = (
        r'MKC: Trying to mark type (?P<mark_type>\d+).',
        r'isd: State changed to DONE'
    )

    states_map = {
        'moving': ['localization'],
        'localization': ['moving', 'marking']
    }

    def __init__(self, df):
        self.df = df

    def _result_key(self, key, i):
        return '{}_{}'.format(key, i)

    def parse(self, start_idx):
        start_time = None
        new_goal_id = None
        # 1. Find new goal start time
        for idx, (timestamp, row) in enumerate(self.df.iloc[start_idx:].iterrows()):
            info = row['info']
            # 1. find new goal started timestamp
            if new_goal_id is None:
                goal_start_p = self.single_goal_keys[0]
                m = re.match(goal_start_p, info)
                if m is None:
                    continue
                new_goal_id = m.group('id')
                start_time = timestamp
                start_idx += idx
                break

        if new_goal_id is None:
            return start_idx, True, []

        internal_id = None
        moving_count = 0
        current_state = 'internal_goal'
        state_p_idx = 0
        res = OrderedDict()
        end_time = None
        for idx, (timestamp, row) in enumerate(self.df.iloc[start_idx:].iterrows()):
            info = row['info']

            if current_state == 'internal_goal':
                if internal_id is None:
                    internal_m = re.match(self.internal_goal_key, info)
                    if internal_m is None:
                        continue
                    internal_id = internal_m.group('internal_id')
                    current_state = 'moving'
            elif current_state == 'moving':
                keys = self.moving_keys
                p = keys[state_p_idx]
                m = re.match(p, info)
                if m is None:
                    continue
                if state_p_idx == 0:
                    moving_count += 1
                start_end = 'start' if state_p_idx == 0 else 'end'
                moving_res_key = self._result_key(current_state, moving_count)
                if moving_res_key not in res:
                    res[moving_res_key] = {}
                res[moving_res_key][start_end] = timestamp
                if state_p_idx == 1:
                    current_state = 'localization'
                state_p_idx = int(not bool(state_p_idx))

            elif current_state == 'localization':
                keys = self.localization_keys
                p = keys[state_p_idx]
                if state_p_idx == 0:
                    p = p(internal_id)
                m = re.match(p, info)
                if m is None:
                    continue

                start_end = 'start' if state_p_idx == 0 else 'end'
                localization_res_key = self._result_key(current_state, moving_count)
                if localization_res_key not in res:
                    res[localization_res_key] = {}
                res[localization_res_key][start_end] = timestamp
                if state_p_idx == 1:
                    # FIXME: find a better way to target state
                    next_state = None
                    for _timestamp, _row in self.df.iloc[idx+start_idx:].iterrows():
                        _info = _row['info']
                        # check whether is new goal
                        m = re.match(self.internal_goal_key, _info)
                        if m is not None:
                            next_state = 'internal_goal'
                            internal_id = None
                            break

                        marking_p = self.marking_keys[0]
                        m = re.match(marking_p, _info)
                        if m is not None:
                            next_state = 'marking'
                            break

                    assert next_state is not None
                    current_state = next_state
                state_p_idx = int(not bool(state_p_idx))

            elif current_state == 'marking':
                keys = self.marking_keys
                p = keys[state_p_idx]
                m = re.match(p, info)
                if m is None:
                    continue

                start_end = 'start' if state_p_idx == 0 else 'end'
                if current_state not in res:
                    res[current_state] = {}
                res[current_state][start_end] = timestamp
                if state_p_idx == 1:
                    start_idx += idx
                    # Find all
                    break
                state_p_idx = int(not bool(state_p_idx))
            else:
                print "Unknown state {}".format(current_state)

        if 'marking' not in res or 'end' not in res['marking']:
            return start_idx, True, []

        end_time = None
        for idx, (timestamp, row) in enumerate(self.df.iloc[start_idx:].iterrows()):
            info = row['info']
            # Waiting goal end?
            goal_end_p = self.single_goal_keys[1]
            goal_end_p = goal_end_p(new_goal_id)
            m = re.match(goal_end_p, info)
            if m is not None:
                end_time = timestamp
                start_idx += idx
                break
        if end_time is None or end_time < res['marking']['end']:
            return start_idx, True, []

        return start_idx, False, [start_time, end_time, res]
