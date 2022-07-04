var ws_json
var hostname
var ip_addr = document.location.hostname

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
        job_data_content += key + ": " + ws_json["job_data"][key] + "</br>"
    }
    var elej_ob = get_id('job_data')
    elej_ob.innerHTML = job_data_content

    // save_data
    save_data_content = get_data(ws_json, "save_data")
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

    // modify button style
    for (const key in ws_json["done"]) {
        get_id(key).setAttribute("class", 'btn btn-info');
    }

}

function get_data(ws_json, first_key) {
    data_content = ""
        // console.log(typeof(ws_json[first_key]))
    if (ws_json[first_key] !== undefined) {
        for (const key in ws_json[first_key]["CAMERA_SHARPNESS"]) {
            if (key != "time") {
                data_content += key + ": " + ws_json[first_key]["CAMERA_SHARPNESS"][key] + ", time :" + time_vis(ws_json[first_key]["CAMERA_SHARPNESS"]["time"]) + "</br>"
            }
        }
        for (const key in ws_json[first_key]["INITIALIZE_SERVO"]) {
            if (key != "time") {
                data_content += key + ": " + ws_json[first_key]["INITIALIZE_SERVO"][key] + ", time :" + time_vis(ws_json[first_key]["INITIALIZE_SERVO"]["time"]) + "</br>"
            }
        }
        for (const key in ws_json[first_key]["CAMERAS_ANGLE"]) {
            if (key != "time") {
                data_content += key + ": " + ws_json[first_key]["CAMERAS_ANGLE"][key] + ", time :" + time_vis(ws_json[first_key]["CAMERAS_ANGLE"]["time"]) + "</br>"
            }
        }
        // for (const key in ws_json[first_key]["angle"]) {
        //     if (key != "time") {
        //         data_content += key + ": " + ws_json[first_key]["angle"][key] + ", time :" + time_vis(ws_json[first_key]["angle"]["time"]) + "</br>"
        //     }
        // }

    }
    return data_content
}


const long_img_socket = new WebSocket("ws://" + ip_addr + "/long_img_ws")
long_img_socket.addEventListener('open', function(event) {
    long_img_socket.send("Hello ws long_img_ws!");
});

long_img_socket.onmessage = function(evt) {
    // convert data to json
    ws_json = eval('(' + evt.data + ')')
    get_id("img1").src = "data:image/jpeg;base64," + ws_json["data"];
    get_id("long_camera_data").innerHTML = "long camera, time: " + time_vis(ws_json["time"]);
}


const short_img_socket = new WebSocket("ws://" + ip_addr + "/short_img_ws")
short_img_socket.addEventListener('open', function(event) {
    short_img_socket.send("Hello ws short_img_ws!");
});

short_img_socket.onmessage = function(evt) {
    ws_json = eval('(' + evt.data + ')')
    get_id("img2").src = "data:image/jpeg;base64," + ws_json["data"];
    get_id("short_camera_data").innerHTML = "short camera, time: " + time_vis(ws_json["time"]);
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
    pom.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(JSON.stringify(ws_json)));

    console.log("download")
        // hostname = get_ws("localhost", "ws_hostname")
    console.log(hostname)
    console.log(JSON.stringify(ws_json))
    var myDate = new Date()
    console.log(myDate.getDate())

    filename = hostname['hostname'] + "_" + myDate.getFullYear() + "_" + myDate.getMonth() + "_" + myDate.getDate() + ".json"

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