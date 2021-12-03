#!/usr/bin/env python
# -*- coding: utf-8 -*-

from common.enums import ConnectStatus

def format_print(devices_status):
    """
    This function will format the print infos in order

    Params:
    -------
        devices_status: list, element is dict {'name': device_name,
            'connected': status}

    Return:
    -------
        Strings containing infos of device status
    """

    def format_result(res):
        if res:
            return "\033[1;32;48m Connected \033[1;37;48m"
        else:
            return "\033[1;31;48m Not Connected \033[1;37;48m"

    ret = ''
    space_width = 4
    max_length = 0  # need to find the longest name first
    for device in devices_status:
        if len(device['name']) > max_length:
            max_length = len(device['name'])
    for device in devices_status:
        ret += device['name']
        ret += (max_length - len(device['name']) + space_width) * ' '
        ret += format_result(device['connected'])
        ret += '\n'

    return ret
