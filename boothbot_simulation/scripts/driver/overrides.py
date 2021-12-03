#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Creates a simple 'overrides' annotation
# Source: https://stackoverflow.com/questions/1167617/in-python-how-do-i-indicate-im-overriding-a-method
# TODO: Can be replaced with `overrides` python package for Python 3.*

def overrides(super_class):
    def overrider(method):
        assert(method.__name__ in dir(super_class))
        return method
    return overrider