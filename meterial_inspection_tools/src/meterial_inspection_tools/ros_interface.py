from boothbot_common.interface_with_type import InterfaceWithType
import std_msgs.msg as stmsgs
import std_srvs.srv as stsrvs
import sensor_msgs.msg as ssmsgs

import boothbot_msgs.msg as bbmsgs
import boothbot_msgs.srv as bbsrvs
import boothbot_grpc.msg as bgmsgs

from meterial_inspection_tools.srv import IMUcontrol
from meterial_inspection_tools.srv import Inclin  # InclinometerControl
from meterial_inspection_tools.srv import CBcontrol # CBControl
from meterial_inspection_tools.srv import Sonarcontrol # SonarControl
from meterial_inspection_tools.srv import DepthCameracontrol # DepthCameraControl



# MODULES_MATERIAL_INSPECTION_DATA = InterfaceWithType('/modules/meterial_inspection/data', stmsgs.String)

# MODULES_MATERIAL_INSPECTION_SRV_CMD = InterfaceWithType('/modules/meterial_inspection/srv_cmd', bbsrvs.Command)

# LDLIDAR_DATA = InterfaceWithType('/ldlidar/data', ssmsgs.LaserScan) # name & type used to instantiate
# LDLIDAR_SRV_CMD = InterfaceWithType('/ldlidar/srv_cmd', bbsrvs.Command)

IMU_DATA = InterfaceWithType('/imu/data', stmsgs.String)
IMU_STATE = InterfaceWithType('/imu/state', stmsgs.String)
IMU_INFO = InterfaceWithType('/imu/info', stmsgs.String)
IMU_SRV_CMD = InterfaceWithType('/imu/srv_cmd', IMUcontrol)
IMU_CONFIGS = InterfaceWithType('/imu/configs',stmsgs.String)

INCLINOMETER_STATE = InterfaceWithType('/inclinometer/state',stmsgs.String)
INCLINOMETER_INFO = InterfaceWithType('/inclinometer/info',stmsgs.String)
INCLINOMETER_CONFIGS = InterfaceWithType('/inclinometer/configs',stmsgs.String)
INCLINOMETER_DATA = InterfaceWithType('/inclinometer/data',stmsgs.String)
INCLINOMETER_SRV_CMD = InterfaceWithType('/inclinometer/srv_cmd',Inclin)

CB_DATA = InterfaceWithType('/cb/data',stmsgs.String)
CB_STATE = InterfaceWithType('/cb/state',stmsgs.String)
CB_INFO =  InterfaceWithType('/cb/info',stmsgs.String)
CB_CONFIGS = InterfaceWithType('/cb/configs',stmsgs.String)
CB_SRV_CMD = InterfaceWithType('/cb/srv_cmd',CBcontrol)

SONAR_DATA = InterfaceWithType('/sonar/data',stmsgs.String)
SONAR_STATE = InterfaceWithType('/sonar/state',stmsgs.String) 
SONAR_INFO = InterfaceWithType('/sonar/info',stmsgs.String)
SONAR_CONFIGS = InterfaceWithType('/sonar/configs',stmsgs.String)
SONAR_SRV_CMD = InterfaceWithType('/sonar/srv_cmd',Sonarcontrol)

DEPTH_DATA = InterfaceWithType('/depth/data', ssmsgs.PointCloud2)
DEPTH_IMAGE = InterfaceWithType('/depth/formatted_image', ssmsgs.Image)
DEPTH_STATE = InterfaceWithType('/depth/state',stmsgs.String) 
DEPTH_INFO = InterfaceWithType('/depth/info',stmsgs.String)
DEPTH_CONFIGS = InterfaceWithType('/depth/configs',stmsgs.String)
DEPTH_SRV_CMD = InterfaceWithType('/depth/srv_cmd',DepthCameracontrol)

msg_dict = {
    # "ldlidar":{
    #     "topic_data": LDLIDAR_DATA,
    #     "srv": LDLIDAR_SRV_CMD
    # },
    "imu":{
        "srv": IMU_SRV_CMD,
        "topic_data": IMU_DATA,
        "topic_state": IMU_STATE,
        "topic_info": IMU_INFO,
        "topic_configs": IMU_CONFIGS,
    },
    
    "inclinometer": {
        "srv" : INCLINOMETER_SRV_CMD,
        "topic_data": INCLINOMETER_DATA,
        "topic_state": INCLINOMETER_STATE,
        "topic_info": INCLINOMETER_INFO,
        "topic_configs": INCLINOMETER_CONFIGS,
    },

    "cb":{
        "srv" : CB_SRV_CMD,
        "topic_data": CB_DATA,
        "topic_state": CB_STATE,
        "topic_info": CB_INFO,
        "topic_configs": CB_CONFIGS,
    },

    "sonar":{
        "srv": SONAR_SRV_CMD,
        "topic_data": SONAR_DATA,
        "topic_state": SONAR_STATE,
        "topic_info": SONAR_INFO,
        "topic_configs": SONAR_CONFIGS,
    },

    "depth_camera":{
        "srv": DEPTH_SRV_CMD,
        "topic_data": DEPTH_DATA,
        "topic_state": DEPTH_STATE,
        "topic_info": DEPTH_INFO,
        "topic_configs": DEPTH_CONFIGS,
        "topic_image": DEPTH_IMAGE,
    }
}