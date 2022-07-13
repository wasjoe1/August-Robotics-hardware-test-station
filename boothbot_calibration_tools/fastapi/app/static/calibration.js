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

display_servos_laser_button = [INITIALIZE_SERVO, CAMERA_SHARPNESS, CAMERA_LASER_ALIGNMENT, CAMERAS_ANGLE, CAMERAS_ALIGNMENT]

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

    // job_data
    job_data_content = ""
    for (const key in ws_json["job_data"]) {
        if (key == "cameras_offset_op_info") {
            if (ws_json["job_data"][key]["small"] == "right") {
                job_data_content += "小相机调右边螺丝"
            } else {
                job_data_content += "小相机调左边螺丝"
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
    save_data_content = get_save_data(ws_json, "save_data")
    var save_data_ele = get_id('need_save_data')
    save_data_ele.innerHTML = save_data_content


    // last_data
    last_data_content = get_data(ws_json, "last_data")
    var save_data_ele = get_id('last_data')
    save_data_ele.innerHTML = last_data_content

    // modify host name
    var ele_hostanme = get_id("hostname")
    ele_hostanme.innerHTML = "主机名: " + ws_json["host_name"]

    // modify current job
    var ele_step = get_id("step")
    ele_step.innerHTML = "当前任务: " + ws_json["step"]

    // client status
    var ele_client_status = get_id("client_status")
    ele_client_status.innerHTML = "电机: " + ws_json["client_status"]["servos"] + ", " + "相机：" + ws_json["client_status"]["cameras"] + "</br>"

    set_button()

    // modify button style


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
        get_id("run_button").innerText = "保存文件到设备配置"
        get_id("done_button").innerText = "完成并重启程序"
    } else {
        get_id("run_button").innerText = "运行"
        get_id("done_button").innerText = "完成"
    }



    for (const key in ws_json["done"]) {
        get_id(key).setAttribute("class", 'btn btn-info');
    }

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
    data_content = ""
        // console.log(typeof(ws_json[first_key]))
    if ((ws_json[first_key] !== undefined)) {
        download_data = ws_json[first_key]
        data_content = get_function_data(first_key, INITIALIZE_SERVO) +
            get_function_data(first_key, CAMERAS_ANGLE) +
            get_function_data(first_key, VERTICAL_SERVO_ZERO) +
            get_function_data(first_key, CAMERA_SHARPNESS)
    }
    return data_content
}

function get_data(ws_json, first_key) {
    data_content = ""
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
    html_data = ""
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
    Y = time.getFullYear() + '-'
    M = (time.getMonth() + 1 < 10 ? '0' + (time.getMonth() + 1) : time.getMonth() + 1) + '-'
    D = time.getDate() + ' '
    h = time.getHours() + ':'
    m = time.getMinutes() + ':'
    s = time.getSeconds()
    return Y + M + D + h + m + s
}


function mode_selection(mode) {
    console.log("send mode :" + mode)
    var url = "http://" + ip_addr + "/mode_select/" + mode
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

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

    filename = ws_json["host_name"] + "_" + myDate.getFullYear() + "_" + myDate.getMonth() + "_" + myDate.getDate() + ".json"

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
    user_manual = get_id("user_manual")
    b = user_manual.innerHTML

    const regex = /\\n|\\r\\n|\\n\\r|\\r/g;
    a = b.split(",")
    s = ''
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
}