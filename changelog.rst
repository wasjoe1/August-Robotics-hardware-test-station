Release Changelog
=================

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_

格式基于 `如何更新维护日志 <https://keepachangelog.com/zh-CN/1.0.0/>`_

[v3.17] - 2023-12-22 
------------------------------
Added
^^^^^

  1. boothbot-calibration-tools:
    a. display two inclinometers data on page
    b. add a parameter to cut blocks of image which token from camera when doing laser alignment
    c. add description of 4 parameters for calibration tools
    d. must input depth camera parameter before doing depth camera calibration 


[v3.11] 
------------------------------
Added
^^^^^

  1. boothbot-simulation:
    a. rostopic /clock for speeding up simulation
  2. tools
    a. add get_incli_data.py for check inclination data


Known Issue
^^^^^^^^^^^
  1. boothbot_calibration_tools:
    a. auto cb calibration in calibration tools, and user can operate on gui.


[v3.9 - unreleased] - 2022-02-06
------------------------------
Added
^^^^^

  1. boothbot-reporter:
    a. auto generate result after each goto_mark.


Known Issue
^^^^^^^^^^^


[v3.8 - released] - 2023-01-18
------------------------------
Added
^^^^^

  1. assemble_tools:
    a. add gs-dtu-aging

  2. boothbot-reporter:
    a. added ability that can process all log in one go, and output a csv file.

  3. boothbot_calibration_tools:
    a. add marking camera roi calibration tools.
    b. add single camera calibration for LNP6.

  4. tools:
    a. add a scripts, this scripts handle mutil compensation file by use one time 8_dir measurement result.

Known Issue
^^^^^^^^^^^
