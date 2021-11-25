from enum import Enum


class ErrCode(Enum):
    """
    ErrCode starts with a specific number to indicate which module it belongs to.
    And after starting number, each ErrCode follows by 3 digits to indicate more
    details error.

    Starting number
    ---------------
    0: No err. OK
    1: GS err.
      10: hardware issue.
        1001: DTU stop working.
        1002: h/v servo error, offline
        1003: h servo error, offline
        1004: v servo error, offline
        1005: large/small camera lost. Failed capture image or captured all zero image
        1006: large camera lost. Failed capture image or captured all zero image
        1007: small camera lost. Failed capture image or captured all zero image
        1008: laser error. Failed to get measurement.
      11: Tracking issue
        1101: large camera cannot find target (instant, no target in current captured image)
        1102: small camera cannot find target (instant, no target in current captured image)
        1103: lost target (after one round finishes, after tracking X times, cannot align target)
      12: Communication issue
        1201: DTU cannot communicate properly. (Didn't get msg for a while, timeout)
      13: Laser measurement result is filtered since it has big different to target distance
        1301: Result filtered error
    2: CB err.
      20: hardware issue.
        2001: DTU stop working.
        2002: h/v servo error, offline
        2003: h servo error, offline
        2004: v servo error, offline
        2005: large/small camera lost. Failed capture image or captured all zero
        2006: large camera lost. Failed capture image or captured all zero
        2007: small camera lost. Failed capture image or captured all zero
        2008: yaw motor error
        2009: yaw encoder error
        2010: pitch motor error
        2011: pitch encoder error
      21: Tracking issue
        2101: large camera cannot find target (instant, no target in current captured image)
        2102: small camera cannot find target (instant, no target in current captured image)
        2103: lost target (after one round finishes, after tracking X times, cannot align target)
      22: Communication issue
        2201: DTU cannot communicate properly. (Didn't get msg for a while, timeout)
      23: Inclinometer issue
        2301: no data from inclinometer (no data update for a while)
    3: Marking error, Ink Stamp / Booth number
       (this one is hard since there is no feedback from ink stamp or xyz platform)
      3001: ink stamp node raise error (Ink stamp node or driver raise error, mostly is IO err)
      3010: bn painter node raise error
      3020: wrong mark (ink stamp / booth number) on the floor
            (potential hardware issue, using marking camera to find out)
      3030: ink camera has error
      3040: too much wrong mark (ink stamp / booth number) on the floor, marking controller will freeze
    4: Chassis
      4001: Chassis is offline, not functional
      4002: Battery is low (Battery returned by movebase)
    5: Obstacle avoiding
      50: lidar
        5001: no raw lidar data (raw data)
        5002: no filtered lidar data (filtered data which is used in obstacle avoiding of Lionel)
        5003: lidar tf (tf goes crazy)
      51: sonar
        5110: front sonar data
        5111: front sonar tf
        5120: rear sonar data
        5121: rear sonar tf
        5130: front lower sonar data
        5131: front lower sonar tf
        5140: side sonar data
        5141: side sonar tf
        5150: front looking down sonar data
        5151: front looking down sonar tf
      5201: costmap (costmap didn't generate or loading failed)
      5301: virtual obstacle publisher (virtual obs publisher node dead)
    6: Localization + Odom
      60: Localization
        6001: cannot localize by GS (related to 11, cannot track Cb)
        6002: cannot localize by CB (related to 21, cannot track GS)
        6003: cannot integrate localized value (date not synced) to odom combined
        6005: Lionel skipped too many goals in a row
      61: Odom
        6101: IMU data (no imu data updating)
        6102: Odom combined is not updated (no data updating. stopped publishing)
    7: Path plan
      7001: global plan cannot find path
      7002: teb cannot find path
    8: date sync (between large PC and small PC on Lionel) [ONLY one error. Using 8000 as errcode]
      8000: date sync is wrong (Need force user stop Lionel and sync date first.)
    9: Goal publisher
      9001: node stopped (goal publisher stopped and no response for any request)
      9002: Bad goal (Sending out bad goals, in most case, it means all goal finished)
      9003: Cannot paint goals list (Existing cannot-be-painted goals, show them)
      9004: Recieve unexpected painted goal feedback (target goal is not same with feedback goal)
    10: Upper controller error state
      10001: Boothbot controller error
      10002: Navigation controller error
    """
    OK = 0
    # GS
    GS_ERR = 1000
    GS_ERR_DTU_OFFLINE = 1001
    GS_ERR_SERVO = 1002 # Either H or V has error
    GS_ERR_H_SERVO = 1003
    GS_ERR_V_SERVO = 1004
    GS_ERR_CAM_OFFLINE = 1005
    GS_ERR_LARGE_CAM_OFFLINE = 1006
    GS_ERR_SMALL_CAM_OFFLINE = 1007
    GS_ERR_LASER_OFFLINE = 1008
    GS_ERR_LARGE_CAM_NO_TARGET = 1101
    GS_ERR_SMALL_CAM_NO_TARGET = 1102
    GS_ERR_LOST_TARGET = 1103
    GS_ERR_DTU_COMM = 1201
    GS_ERR_LASER_RESULT_FILTERED = 1301  # laser result vs target dist > 2m
    # CB
    CB_ERR = 2000
    CB_ERR_DTU_OFFLINE = 2001
    CB_ERR_SERVO = 2002 # Either H or V has error
    CB_ERR_H_SERVO = 2003
    CB_ERR_V_SERVO = 2004
    CB_ERR_CAM_OFFLINE = 2005
    CB_ERR_LARGE_CAM_OFFLINE = 2006
    CB_ERR_SMALL_CAM_OFFLINE = 2007
    CB_ERR_H_MOTOR = 2008
    CB_ERR_H_ENCODER = 2009
    CB_ERR_V_MOTOR = 2010
    CB_ERR_V_ENCODER = 2011
    CB_ERR_LARGE_CAM_NO_TARGET = 2101
    CB_ERR_SMALL_CAM_NO_TARGET = 2102
    CB_ERR_LOST_TARGET = 2103
    CB_ERR_DTU_COMM = 2201
    CB_ERR_INCLINOMETER = 2301
    # Ink stamp / Booth Number
    MK_ERR = 3000
    IS_ERR_INTERNAL_ERR = 3001
    BN_ERR_INTERNAL_ERR = 3010
    MK_ERR_WRONG_MARK = 3020
    MK_ERR_CAM_NO_DATE = 3030
    MK_ERR_MARK_FAILURE = 3040
    # Chassis
    CH_ERR = 4000
    CH_ERR_OFFLINE = 4001
    CH_ERR_LOW_BATTERY = 4002
    # Obstacle avoiding
    OA_ERR = 5000
    OA_ERR_LIDAR_RAW = 5001
    OA_ERR_LIDAR_FILTERED = 5002
    OA_ERR_LIDAR_TF = 5003
    OA_ERR_SONAR_FRONT_DATA = 5110
    OA_ERR_SONAR_FRONT_TF = 5111
    OA_ERR_SONAR_REAR_DATA = 5120
    OA_ERR_SONAR_REAR_TF = 5121
    OA_ERR_SONAR_FRONT_LOW_DATA = 5130
    OA_ERR_SONAR_FRONT_LOW_TF = 5131
    OA_ERR_SONAR_SIDE_DATA = 5140
    OA_ERR_SONAR_SIDE_TF = 5141
    OA_ERR_SONAR_FRONT_LOOKDOWN_DATA = 5150
    OA_ERR_SONAR_FRONT_LOOKDOWN_TF = 5151
    OA_ERR_SONAR = 5160 # Summary sonar error means 5110 ~ 5151, we need split if want details
    OA_ERR_COSTMAP = 5201
    OA_ERR_VIRTUAL_OBSTACLE_PUB = 5301
    # Localization + odom
    LO_ERR = 6000
    LO_ERR_GS_LOCALIZE_FAIL = 6001
    LO_ERR_CB_LOCALIZE_FAIL = 6002
    LO_ERR_INTEGRATE_LOCALIZED_VALUE = 6003
    LO_ERR_GBM_LOCALIZE_FAIL = 6004
    LO_ERR_SKIPPED_TOO_MANY = 6005
    LO_ERR_IMU = 6101
    LO_ERR_ODOM_COMBINED = 6102
    LO_ERR_INCLINOMETER = 6103
    # Path Plan
    PP_ERR = 7000
    PP_ERR_GLOBAL = 7001
    PP_ERR_TEB = 7002
    # Date sync
    DS_ERR = 8000
    # Goal publisher
    GP_ERR = 9000
    GP_ERR_NODE_DEAD = 9001
    GP_ERR_BAD_GOAL = 9002
    GP_ERR_WONT_PAINT_GOALS = 9003
    GP_ERR_UNEXPECTED_GOAL_FEEDBACK = 9004
    # Upper controller error
    BBC_ERR = 10001
    NAVC_ERR = 10002


# To make things easier and simpler, create errcode and msg map
# which will use `name` of `ErrCode(Enum)` as value and `value`
# of `ErrCode(Enum)` as key
errcode_msg_map = dict([
    (e.value,
     " ".join(e.name.split("_")) if e != ErrCode.OK else '')
    for e in ErrCode
])

# To make error code frame id consistency, we define all
# frame id which will be used in error code topic.
# The frame id will be show in GUI dashboard as well.
GS_FID = 'Guiding Station'      # 1000, 1103
GS_DTU_FID = 'GS DTU'           # 1001, 1201
GS_SERVO_FID = 'GS Servo'       # 1002, 1003
GS_CAM_FID = 'GS Camera'        # 1004, 1005, 1101, 1102
GS_LASER_FID = 'GS Laser'       # 1106, 1301

CB_FID = 'Camera Beacon'        # 2000
CB_DTU_FID = 'CB DTU'           # 2001, 2201
CB_SERVO_FID = 'CB Servo'       # 2002, 2003
CB_CAM_FID = 'CB Camera'        # 2004, 2005, 2101, 2102

IS_FID = 'Ink Stamp'            # 3000, 3001
BN_FID = 'Booth Number'         # 3000, 3010
MK_ERR_WRONG_MARK_FID = 'Wrong Mark' # 3020
MK_CAM_FID = 'Mark Camera'      # 3030
MK_ERR_MARK_FAILURE_FID = 'Mark Failure' # 3040

CH_FID = 'Chassis'              # 400x

OA_FID = 'Obstacle Avoiding'    # 5000
LIDAR_FID = 'Lidar'             # 5001, 5002, 5003
SONAR_FID = 'Sonar'             # 51xx
COSTMAP_FID = 'Costmap'         # 5201
VO_PUB_FID = 'Virtual Obstacle' # 5301

LO_FID = 'Localization'         # 6000, 6001, 6002, 6003
IMU_FID = 'IMU'                 # 6101
ODOM_COMBINED_FID = 'Odom Combined' # 6102
INCLINOMETER_FID = 'Inclinometer' # 6103

PP_FID = 'Path Plan'            # 700x
DS_FID = 'Date Sync'            # 8000

GP_FID = 'Goal Sender'          # 900x

BBC_FID = 'BBC'                 # 10001
NAVC_FID = 'NAVC'               # 10002
