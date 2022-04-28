#!/usr/bin/env python
#encoding=utf-8

from assemble_tools.get_key import GetKey

if __name__ == "__main__":
    get_key = GetKey()
    print("Check get key: {}".format(get_key.get_key()))
    while True:
        key = get_key.get_key()
        if key:
            print("Key is: {}".format(key))
        if key.lower() == "e":
            print("exiting....")
            break

    get_key.end_get_key()
