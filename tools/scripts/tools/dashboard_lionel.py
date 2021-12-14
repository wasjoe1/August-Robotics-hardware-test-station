#!/usr/bin/env python
import redis

from common.redis_keys import BOOTHBOT_STATE, NAVC_STATE, MK_STATE, CB_STATE
from common.errcode import BBC_FID, NAVC_FID, IS_FID, CB_FID, BN_FID
from boothbot_common.settings import MSR_HOST, REDIS_PORT, IS_BOOTHNUMBER

from dashboard_states import DashboardStates


class LionelStates(DashboardStates):

    show_state_lst = [
        (BBC_FID, BOOTHBOT_STATE),
        (NAVC_FID, NAVC_STATE),
        (BN_FID if IS_BOOTHNUMBER else IS_FID, MK_STATE),
        (CB_FID, CB_STATE),
    ]


if __name__=="__main__":
    R = redis.StrictRedis(host=MSR_HOST, port=REDIS_PORT)
    ls = LionelStates(R=R)
    ls.terminal_loop_display()
