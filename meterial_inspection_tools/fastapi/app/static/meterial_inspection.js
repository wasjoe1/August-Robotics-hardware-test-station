// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var lidar_data = []

var url = window.location.href
const regex = "http://(.*)/step/(.*)"
const found = url.match(regex)
current_step = found[2]
console.log("current step: ", current_step)

// ------------------------------------------------------------------------------------------------
// Functions
function get_ws(ip_addr, route, elementId) {
    const ws = new WebSocket("ws://" + ip_addr + route) // route == /imu_smt
    ws.addEventListener('open', function(event) {
        console.log(`${route} socket was opended`)
        ws.send('Hello ws data!');
    });
    ws.onmessage = function(evt) {
        console.log(`JS script receive message from backend from ${route} socket`)
        console.log(`data from ${route} socket: ${evt.data}`)
    
        document.getElementById(elementId).textContent = evt.data 
        return evt.data
    }
}

function convert_data_to_pointcloud(angle_increment, ranges){
    start_ang = 0
    for (let i = 0; i < ranges.length; i++) {
        ang = start_ang + i*angle_increment
        lidar_data.push([ranges[i]*Math.cos(ang), ranges[i]*Math.sin(ang)]);
    }
}

function command(cmd) {
    var cmd_dict = {}
    cmd_dict[current_step] = cmd
    cmd_str = JSON.stringify(cmd_dict)
    // cmd_str = current_step + "_" + cmd
    console.log("send cmd: " + cmd_str)
    var url = "http://" + ip_addr + "/command/" + cmd_str
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

function get_id(id) {
    return document.getElementById(id)
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
// open web socket connection for /data (imu data) => for the imu readings
get_ws(ip_addr, "/imu_data", "responseData-data")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /state => for the current state
get_ws(ip_addr, "/imu_state", "responseData-state")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /info => for user status
get_ws(ip_addr, "/imu_info", "responseData-info")

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// OTHERS XX NOT IN USED XX
// const long_img_socket = new WebSocket("ws://" + ip_addr + "/long_img_ws")
// long_img_socket.addEventListener('open', function(event) {
//     long_img_socket.send("Hello ws long_img_ws!");
// });

// long_img_socket.onmessage = function(evt) {
//     // convert data to json
//     // console.log(evt.data)
//     if (evt.data !== "") {
//         ws_json = eval('(' + evt.data + ')')
//         get_id("img1").src = "data:image/jpeg;base64," + ws_json["data"];
//         get_id("long_camera_data").innerHTML = "long camera, time: " + time_vis(ws_json["time"]);
//     }
// }

// document.onkeydown=function(event){
//     var e = event || window.event || arguments.callee.caller.arguments[0];
//     console.log(e.keyCode)
//     if(e && e.keyCode==76){ 
//         console.log("LASER_ON")
//         command('LASER_ON')
//       }
//     if(e && e.keyCode==78){ 
//         console.log("DISABLE")
//         command('SERVOS_DISABLE')
//        }            
//      if(e && e.keyCode==82){ 
//         console.log("ENABLE")
//         command('SERVOS_ENABLE')
//     }
//     if(e && e.keyCode==66){ 
//         console.log("USE_SHORT_CAMERA")
//         command('USE_SHORT_CAMERA')
//     }
//     if(e && e.keyCode==83){ 
//         console.log("USE_LONG_CAMERA")
//         command('USE_LONG_CAMERA')
//     }
// }; 

// ws_json = eval('(' + evt.data + ')')
// if ((current_step in ws_json)){
//     // imu display
//     ws_json = ws_json[current_step]
//     convert_data_to_pointcloud(ws_json["angle_increment"], ws_json["ranges"])
//     find_element()
//     drawLidar(lidar_data)
//     lidar_data = []
// }
