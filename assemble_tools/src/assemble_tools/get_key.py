#!/usr/bin/env python
#encoding=utf-8
import sys, select, termios, tty

class GetKey(object):
    _initialized = False
    _settings = None
    def __init__(self):
        self.init_get_key()

    def init_get_key(self):
        self._settings = termios.tcgetattr(sys.stdin)
        self._initialized = True

    def get_key(self):
        if self._initialized is False:
            print("Not initialized!")
            return None
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    def end_get_key(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
