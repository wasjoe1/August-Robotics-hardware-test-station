// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var INITIALIZE_SERVO = "INITIALIZE_SERVO"
var CAMERA_SHARPNESS = "CAMERA_SHARPNESS"
var CAMERA_LASER_ALIGNMENT = "CAMERA_LASER_ALIGNMENT"
var CAMERAS_ALIGNMENT = "CAMERAS_ALIGNMENT"
var CAMERAS_ANGLE = "CAMERAS_ANGLE"
var VERTICAL_SERVO_ZERO = "VERTICAL_SERVO_ZERO"
var IMU_CALIBRATION = "IMU_CALIBRATION"
var HORIZONTAL_OFFSET = "HORIZONTAL_OFFSET"

// var default_lang = 1
var server_lang
var cur_lang = 1

// var display_servos_laser_button = [INITIALIZE_SERVO, CAMERA_SHARPNESS, CAMERA_LASER_ALIGNMENT, CAMERAS_ANGLE, CAMERAS_ALIGNMENT]

function get_ws(ip, route, data) {
    const ws = new WebSocket("ws://" + ip_addr + "/" + route)
    ws.addEventListener('open', function(event) {
        ws.send('Hello ws data!');
    });
    ws.onmessage = function(evt) {
        console.log(evt.data)
        ws_json = eval('(' + evt.data + ')')
        console.log(ws_json)
        data = ws_json
        return data
    }

}


const data_socket = new WebSocket("ws://" + ip_addr + "/data")
data_socket.addEventListener('open', function(event) {
    data_socket.send('Hello ws data!');
});

function get_id(id) {
    return document.getElementById(id)
}

data_socket.onmessage = function(evt) {
    // convert data to json
    console.log(evt.data)
    ws_json = eval('(' + evt.data + ')')

    // modify host name
    var ele_hostanme = get_id("hostname")
    ele_hostanme.innerHTML = get_lang("hostname") + " : " + ws_json["host_name"]
    hostname = ws_json["host_name"]
    var is_gs = hostname.includes("GS")

    // job_data
    var job_data_content = ""
    for (const key in ws_json["job_data"]) {
        if (key == "cameras_offset_op_info") {
            if (ws_json["job_data"][key]["small"] == "right") {
                // job_data_content += "小相机调右边螺丝" 
                if (is_gs == true){
                    job_data_content += get_lang("short_tun_right_tight")
                } else {
                    job_data_content += get_lang("long_tun_left_tight")
                }
            } else {
                // job_data_content += "小相机调左边螺丝"
                if (is_gs == true){
                    job_data_content += get_lang("short_tun_left_tight")
                } else {
                    job_data_content += get_lang("long_tun_right_tight")
                }
            }
        } else {
            if (key == "measurement_time") { job_data_content += key + ": " + time_vis(ws_json["job_data"][key]) + "</br>" } else {
                job_data_content += key + ": " + ws_json["job_data"][key] + "</br>"
            }
        }
    }

    var elej_ob = get_id('job_data')
    elej_ob.innerHTML = job_data_content

    // save_data
    var save_data_content = get_save_data(ws_json, "save_data")
    var save_data_ele = get_id('need_save_data')
    save_data_ele.innerHTML = save_data_content


    // last_data
    var last_data_content = get_data(ws_json, "last_data")
    var save_data_ele = get_id('last_data')
    save_data_ele.innerHTML = last_data_content

    // modify current job
    var ele_step = get_id("current_task")
    ele_step.innerHTML = get_lang("current_task") + " : " + ws_json["step"]

    // client status
    var ele_client_status = get_id("client_status")
    ele_client_status.innerHTML = get_lang("servos") + ":" + ws_json["client_status"]["servos"] + ", " +
        get_lang("cameras") + ":" + ws_json["client_status"]["cameras"] + "</br>"

    set_button()

    // modify button style


}

function get_lang(key) {
    return lang[key][cur_lang]
}

function set_button() {
    // console.log(step)
    [...document.getElementsByClassName("cameras")].forEach(
        (element, index, array) => {
            if (ws_json["step"] == "CAMERA_SHARPNESS") {
                element.style.display = "block"
            } else {
                element.style.display = "none"
            }
        }
    );
    // [...document.getElementsByClassName("servos_laser_operate")].forEach(
    //     (element, index, array) => {
    //         if (display_servos_laser_button.includes(ws_json["step"])) {
    //             element.classList.remove("disabled")
    //         } else {
    //             element.classList.add("disabled")
    //         }
    //     }
    // );


    if (ws_json["step"] == INITIALIZE_SERVO) {
        get_id("run").innerText = get_lang("save_data_to_device")
        get_id("done").innerText = get_lang("restart")
    } else if (ws_json["step"] == IMU_CALIBRATION) {
        get_id("run").innerText = get_lang("imu_cali")
        get_id("done").innerText = get_lang("imu_save")
    } else {
        get_id("run").innerText = get_lang("run")
        get_id("done").innerText = get_lang("done")
    }


    // for (const key in ws_json["done"]) {
    //     get_id(key).setAttribute("class", 'btn btn-info');
    // }

    // let elements = document.getElementsByClassName('cameras');
    // long = get_id("camera_select_long")
    // short = get_id("camera_select_short")
    // if (ws_json["step"] !== "CAMERA_SHARPNESS") {
    //     long.classList.add("disabled")
    //     short.classList.add("disabled")
    // } else {
    //     long.classList.remove("disabled")
    //     short.classList.remove("disabled")
    // }

    // if (step == "INITIALIZE_SERVO") {
    //     if (ws_json["client_status"]["servos"]) {

    //     }
    // }
}

function get_save_data(ws_json, first_key) {
    var data_content = ""
        // console.log(typeof(ws_json[first_key]))
    if ((ws_json[first_key] !== undefined)) {
        download_data = ws_json[first_key]
        data_content = get_function_data(first_key, INITIALIZE_SERVO) +
            get_function_data(first_key, CAMERAS_ANGLE) +
            get_function_data(first_key, CAMERAS_ALIGNMENT) +
            get_function_data(first_key, VERTICAL_SERVO_ZERO) +
            get_function_data(first_key, CAMERA_SHARPNESS) +
            get_function_data(first_key, HORIZONTAL_OFFSET) +
            get_function_data(first_key, IMU_CALIBRATION)
    }
    return data_content
}

function get_data(ws_json, first_key) {
    var data_content = ""
        // console.log(typeof(ws_json[first_key]))
    if ((ws_json[first_key] !== undefined)) {
        data_content = get_function_data(first_key, INITIALIZE_SERVO) +
            get_function_data(first_key, CAMERAS_ANGLE) +
            get_function_data(first_key, CAMERA_SHARPNESS) +
            get_function_data(first_key, VERTICAL_SERVO_ZERO)
    }
    return data_content
}

function get_function_data(first_key, title) {
    var html_data = ""
        // console.log(Object.keys(ws_json[first_key][title]).length)
    if (ws_json[first_key][title] !== undefined) {
        for (const key in ws_json[first_key][title]) {
            if (key != "measurement_time") {
                html_data += key + ": " + ws_json[first_key][title][key] + ", time :" + time_vis(ws_json[first_key][title]["measurement_time"]) + "</br>"
            }
        }
        return html_data
    } else {
        return ""
    }
}

const long_img_socket = new WebSocket("ws://" + ip_addr + "/long_img_ws")
long_img_socket.addEventListener('open', function(event) {
    long_img_socket.send("Hello ws long_img_ws!");
});

long_img_socket.onmessage = function(evt) {
    // convert data to json
    // console.log(evt.data)
    if (evt.data !== "") {
        ws_json = eval('(' + evt.data + ')')
        get_id("img1").src = "data:image/jpeg;base64," + ws_json["data"];
        get_id("long_camera_data").innerHTML = "long camera, time: " + time_vis(ws_json["time"]);
    }
}


const short_img_socket = new WebSocket("ws://" + ip_addr + "/short_img_ws")
short_img_socket.addEventListener('open', function(event) {
    short_img_socket.send("Hello ws short_img_ws!");
});

short_img_socket.onmessage = function(evt) {
    if (evt.data !== "") {
        ws_json = eval('(' + evt.data + ')')
        get_id("img2").src = "data:image/jpeg;base64," + ws_json["data"];
        get_id("short_camera_data").innerHTML = "short camera, time: " + time_vis(ws_json["time"]);
    }
}

function time_vis(timestamp) {
    timestamp = Math.trunc(timestamp * 1000)
    var time = new Date(timestamp)
    var Y = time.getFullYear() + '-'
    var M = (time.getMonth() + 1 < 10 ? '0' + (time.getMonth() + 1) : time.getMonth() + 1) + '-'
    var D = time.getDate() + ' '
    var h = time.getHours() + ':'
    var m = time.getMinutes() + ':'
    var s = time.getSeconds()
    return Y + M + D + h + m + s
}

function get_lang_requeset() {
    console.log("send get_lang")
    var url = "http://" + ip_addr + "/get_lang"
    var request = new XMLHttpRequest()
    request.onreadystatechange = function() {
        if (request.readyState === 4) {
            get_lang_request_cb(request.response);
        }
    }
    request.open("GET", url)
    request.send()
}

function get_lang_request_cb(response) {
    console.log(response)
        // server_lang = Integer.parseInt([response])
    server_lang = Number(response.substr(1, 1))
    console.log(typeof server_lang)
        // console.log(Number(response))
    console.log(server_lang)
    update_lang()
        // refresh_page_once(cur_lang)
}

function update_lang() {
    console.log("updating lang")
    console.log(server_lang)
    console.log(cur_lang)
    if (server_lang != cur_lang) {
        refresh_page_once(server_lang)
        cur_lang = server_lang
    }
}

function switch_lang() {
    if (cur_lang == 0) {
        cur_lang = 1
    } else {
        cur_lang = 0
    }
    console.log("send cur_lang :" + cur_lang)
    var url = "http://" + ip_addr + "/switch_lang/" + cur_lang
    var request = new XMLHttpRequest()
    request.onreadystatechange = function() {
        if (request.readyState === 4) {
            request_cb(request.response);
        }
    }
    request.open("GET", url)
    request.send()
    refresh_page_once(cur_lang)
}

function request_cb(response) {
    console.log(unescape(response))
    server_lang = cur_lang
        // console.log("xml callback")
    updata_user_manual(response)
}

function refresh_page_once(l) {
    for (ele in refresh_page_once_list) {
        console.log(lang, ele)
        console.log(refresh_page_once_list[ele])
        console.log(lang[refresh_page_once_list[ele]])
        get_id(refresh_page_once_list[ele]).innerText = lang[refresh_page_once_list[ele]][l]
    }
}

// function mode_selection(mode) {
//     console.log("send mode :" + mode)
//     var url = "http://" + ip_addr + "/mode_select/" + mode
//     var request = new XMLHttpRequest()
//     request.open("GET", url)
//     request.send()
// }

function command(cmd) {
    console.log("send cmd :" + cmd)
    var url = "http://" + ip_addr + "/command/" + cmd
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

function download() {
    var pom = document.createElement('a');
    pom.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(JSON.stringify(download_data)));

    console.log("download")
    console.log(download_data)
        // hostname = get_ws("localhost", "ws_hostname")
        // console.log(hostname)
        // console.log(JSON.stringify(ws_json))
    var myDate = new Date()
    console.log(myDate.getDate())

    var filename = hostname + "_" + myDate.getFullYear() + "_" + myDate.getMonth() + "_" + myDate.getDate() + ".json"

    pom.setAttribute('download', filename);

    if (document.createEvent) {
        var event = document.createEvent('MouseEvents');
        event.initEvent('click', true, true);
        pom.dispatchEvent(event);
    } else {
        pom.click();
    }
}

function rollImg(o) {
    var zoom = parseInt(o.style.zoom) || 100
    zoom += event.wheelDelta / 12
    if (zoom > 0) o.style.zoom = zoom + '%'
    return false
}

function after_load() {
    var user_manual = get_id("user_manual")
    var b = user_manual.innerHTML

    const regex = /\\n|\\r\\n|\\n\\r|\\r/g;
    var a = b.split(",")
    var s = ''
    for (let x = 0; x < a.length; x++) {
        console.log(a[x])
        ele = a[x].replace(regex, '<br>');
        ele = ele.replace("[", '');
        ele = ele.replace("]", '');
        ele = ele.replace(/'/g, '');
        console.log(ele)
        s = s + ele
    }
    user_manual.innerHTML = s
    get_lang_requeset()
        // update_lang()
}

function updata_user_manual(data) {
    var user_manual = get_id("user_manual")
    var b = data

    const regex = /\\n|\\r\\n|\\n\\r|\\r/g;
    var a = b.split(",")
    var s = ''
    for (let x = 0; x < a.length; x++) {
        console.log(unescape(a[x]))
        ele = a[x].replace(regex, '<br>');
        ele = ele.replace("[", '');
        ele = ele.replace("]", '');
        ele = ele.replace('"', '');
        ele = ele.replace('"', '');
        ele = ele.replace(/'/g, '');
        // console.log(decode_utf8(ele))
        s = s + ele
    }
    user_manual.innerHTML = s
}

// function decode_utf8(s) {
//     return decodeURIComponent(escape(s));
// }
document.onkeydown=function(event){
    var e = event || window.event || arguments.callee.caller.arguments[0];
    if(e && e.keyCode==76){ 
        console.log("LASER_ON")
        command('LASER_ON')
      }
    if(e && e.keyCode==78){ 
        console.log("DISABLE")
        command('SERVOS_DISABLE')
       }            
     if(e && e.keyCode==82){ 
        console.log("ENABLE")
        command('SERVOS_ENABLE')
    }
    if(e && e.keyCode==66){ 
        console.log("ENABLE")
        command('USE_SHORT_CAMERA')
    }
    if(e && e.keyCode==83){ 
        console.log("ENABLE")
        command('USE_LONG_CAMERA')
    }
}; 