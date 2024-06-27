from boothbot_common.interface_with_type import InterfaceWithType
import std_msgs.msg as stmsgs
import std_srvs.srv as stsrvs
import sensor_msgs.msg as ssmsgs

import boothbot_msgs.msg as bbmsgs
import boothbot_msgs.srv as bbsrvs
import boothbot_grpc.msg as bgmsgs

from meterial_inspection_tools.srv import IMUControl #IMUcontrol
from meterial_inspection_tools.srv import InclinometerControl #Inclin
from meterial_inspection_tools.srv import CBControl #CBcontrol
from meterial_inspection_tools.srv import SonarControl #Sonarcontrol
#from meterial_inspection_tools.srv import DepthCameraControl #DepthCameracontrol
from meterial_inspection_tools.srv import GetButton, GetButtonBaudrate, GetButtonUnitID,GetButtonModel
#from meterial_inspection_tools.srv import GetButtonBaudrate




# MODULES_MATERIAL_INSPECTION_DATA = InterfaceWithType('/modules/meterial_inspection/data', stmsgs.String)

# MODULES_MATERIAL_INSPECTION_SRV_CMD = InterfaceWithType('/modules/meterial_inspection/srv_cmd', bbsrvs.Command)

# LDLIDAR_DATA = InterfaceWithType('/ldlidar/data', ssmsgs.LaserScan) # name & type used to instantiate
# LDLIDAR_SRV_CMD = InterfaceWithType('/ldlidar/srv_cmd', bbsrvs.Command)



IMU_DATA = InterfaceWithType('/imu/data', stmsgs.String)
IMU_STATE = InterfaceWithType('/imu/state', stmsgs.String)
IMU_INFO = InterfaceWithType('/imu/info', stmsgs.String)
IMU_INFO_CHINESE = InterfaceWithType('/imu/info_chinese', stmsgs.String)
IMU_CONFIGS = InterfaceWithType('/imu/configs',stmsgs.String)
IMU_CONFIGS_CHINESE = InterfaceWithType('/imu/configs_chinese',stmsgs.String)
IMU_SRV_CMD = InterfaceWithType('/imu/srv_cmd', GetButtonBaudrate) 
IMU_DATA_CHECK = InterfaceWithType('/imu/reading_checker', stmsgs.String)

INCLINOMETER_DATA = InterfaceWithType('/inclinometer/data',stmsgs.String)
INCLINOMETER_STATE = InterfaceWithType('/inclinometer/state',stmsgs.String)
INCLINOMETER_INFO = InterfaceWithType('/inclinometer/info',stmsgs.String)
INCLINOMETER_INFO_CHINESE =InterfaceWithType('/inclinometer/info_chinese',stmsgs.String)
INCLINOMETER_CONFIGS = InterfaceWithType('/inclinometer/configs',stmsgs.String)
INCLINOMETER_CONFIGS_CHINESE = InterfaceWithType('/inclinometer/configs_chinese',stmsgs.String)
INCLINOMETER_SRV_CMD = InterfaceWithType('/inclinometer/srv_cmd', GetButtonBaudrate) 
INCLINOMETER_DATA_CHECK = InterfaceWithType('/inclinometer/reading_checker', stmsgs.String)

CB_DATA = InterfaceWithType('/cb/data',stmsgs.String)
CB_STATE = InterfaceWithType('/cb/state',stmsgs.String)
CB_INFO =  InterfaceWithType('/cb/info',stmsgs.String)
CB_INFO_CHINESE = InterfaceWithType('/cb/info_chinese',stmsgs.String)
CB_CONFIGS = InterfaceWithType('/cb/configs',stmsgs.String)
CB_CONFIGS_CHINESE = InterfaceWithType('/cb/configs_chinese',stmsgs.String)
CB_SRV_CMD = InterfaceWithType('/cb/srv_cmd', GetButtonUnitID) 
CB_DATA_CHECK = InterfaceWithType('cb/reading_checker',stmsgs.String)

SONAR_DATA = InterfaceWithType('/sonar/data',stmsgs.String)
SONAR_STATE = InterfaceWithType('/sonar/state',stmsgs.String) 
SONAR_INFO = InterfaceWithType('/sonar/info',stmsgs.String)
SONAR_INFO_CHINESE = InterfaceWithType('/sonar/info_chinese',stmsgs.String)
SONAR_CONFIGS = InterfaceWithType('/sonar/configs',stmsgs.String)
SONAR_CONFIGS_CHINESE = InterfaceWithType('/sonar/configs_chinese',stmsgs.String)
SONAR_SRV_CMD = InterfaceWithType('/sonar/srv_cmd',GetButtonUnitID) 

DEPTH_DATA = InterfaceWithType('/depth/data', stmsgs.String)
DEPTH_STATE = InterfaceWithType('/depth/state',stmsgs.String) 
DEPTH_INFO = InterfaceWithType('/depth/info',stmsgs.String)
DEPTH_INFO_CHINESE = InterfaceWithType('/depth/info_chinese',stmsgs.String)
DEPTH_SRV_CMD = InterfaceWithType('/depth/srv_cmd',GetButton)

LIDAR_DATA_LASERSCAN = InterfaceWithType('/lidar/data_laserscan',ssmsgs.LaserScan)
LIDAR_DATA_SCAN = InterfaceWithType('/scan',ssmsgs.LaserScan)
#LIDAR_DATA_POINTCLOUD = InterfaceWithType('lidar/data_pointcloud',ssmsgs.PointCloud2)
LIDAR_DATA_POINTCLOUD = InterfaceWithType('lidar/data_pointcloud',stmsgs.String)
LIDAR_STATE = InterfaceWithType('lidar/state',stmsgs.String)
LIDAR_INFO = InterfaceWithType('lidar/info',stmsgs.String)
LIDAR_INFO_CHINESE = InterfaceWithType('lidar/info/chinese',stmsgs.String)
LIDAR_SRV_CMD = InterfaceWithType('lidar/srv_cmd',GetButtonModel) 



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
        "topic_info_chinese": IMU_INFO_CHINESE,
        "topic_configs": IMU_CONFIGS,
        "topic_configs_chinese": IMU_CONFIGS_CHINESE,
        "topic_data_checker": IMU_DATA_CHECK,
    },
    
    "inclinometer": {
        "srv" : INCLINOMETER_SRV_CMD,
        "topic_data": INCLINOMETER_DATA,
        "topic_state": INCLINOMETER_STATE,
        "topic_info": INCLINOMETER_INFO,
        "topic_info_chinese": INCLINOMETER_INFO_CHINESE,
        "topic_configs": INCLINOMETER_CONFIGS,
        "topic_configs_chinese": INCLINOMETER_CONFIGS_CHINESE,
        "topic_data_checker": INCLINOMETER_DATA_CHECK,
    },

    "cb":{
        "srv" : CB_SRV_CMD,
        "topic_data": CB_DATA,
        "topic_state": CB_STATE,
        "topic_info": CB_INFO,
        "topic_info_chinese": CB_INFO_CHINESE,
        "topic_configs": CB_CONFIGS,
        "topic_configs_chinese": CB_CONFIGS_CHINESE,
        "topic_data_checker": CB_DATA_CHECK,
    },

    "sonar":{
        "srv": SONAR_SRV_CMD,
        "topic_data": SONAR_DATA,
        "topic_state": SONAR_STATE,
        "topic_info": SONAR_INFO,
        "topic_info_chinese": SONAR_INFO_CHINESE,
        "topic_configs": SONAR_CONFIGS,
        "topic_configs_chinese": SONAR_CONFIGS_CHINESE,
    },

    "depth":{
        "srv": DEPTH_SRV_CMD,
        "topic_data": DEPTH_DATA,
        "topic_state": DEPTH_STATE,
        "topic_info": DEPTH_INFO,
        "topic_info_chinese": DEPTH_INFO_CHINESE,
    },

   "lidar": {
        "srv": LIDAR_SRV_CMD,
        "topic_data_laserscan": LIDAR_DATA_LASERSCAN,
        "topic_data_pointcloud": LIDAR_DATA_POINTCLOUD,
        "scan": LIDAR_DATA_SCAN,
        "topic_data": LIDAR_DATA_POINTCLOUD, # for now its point cloud
        "topic_state": LIDAR_STATE,
        "topic_info": LIDAR_INFO,
        "topic_info_chinese": LIDAR_INFO_CHINESE,
    }
}
