#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob


def main():
    ext = 'py'
    files = glob.glob('*/**/*.{}'.format(ext))

    for fpath in files:
        with open(fpath, 'rb+') as f:
            content = f.read()
            f.seek(0)
            f.write(content.replace(b'\r', b''))
            f.truncate()

if __name__=='__main__':
    print("Will try to remove all `\r` character in all py files.")
    confirm = raw_input('Continue? [y/N]: ')
    if confirm in ['y', 'yes', 'Yes', 'YES']:
        print("Start")
        main()
        print("Done")
    else:
        print("Leave")
