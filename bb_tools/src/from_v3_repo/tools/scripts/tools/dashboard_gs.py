#!/usr/bin/env python
import redis

from common.redis_keys import GS_STATE
from common.errcode import GS_FID

from dashboard_states import DashboardStates


class GSStates(DashboardStates):

    show_state_lst = [
        (GS_FID, GS_STATE),
    ]


if __name__=="__main__":
    R = redis.StrictRedis()
    gs = GSStates(R=R)
    gs.terminal_loop_display()
