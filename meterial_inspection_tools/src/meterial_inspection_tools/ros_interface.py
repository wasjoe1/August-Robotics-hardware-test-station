from boothbot_common.interface_with_type import InterfaceWithType
import std_msgs.msg as stmsgs
import std_srvs.srv as stsrvs
import sensor_msgs.msg as ssmsgs

import boothbot_msgs.msg as bbmsgs
import boothbot_msgs.srv as bbsrvs
import boothbot_grpc.msg as bgmsgs


# MODULES_MATERIAL_INSPECTION_DATA = InterfaceWithType('/modules/meterial_inspection/data', stmsgs.String)

# MODULES_MATERIAL_INSPECTION_SRV_CMD = InterfaceWithType('/modules/meterial_inspection/srv_cmd', bbsrvs.Command)

LDLIDAR_DATA = InterfaceWithType('/ldlidar/data', ssmsgs.LaserScan)
LDLIDAR_SRV_CMD = InterfaceWithType('/ldlidar/srv_cmd', bbsrvs.Command)



msg_dict = {
    "ldlidar":{
        "topic":LDLIDAR_DATA,
        "srv":LDLIDAR_SRV_CMD
    }
}