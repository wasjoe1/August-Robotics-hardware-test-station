var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var url = window.location.href
const regex = "http://(.*)/step/(.*)"
const found = url.match(regex)
current_step = found[2]
console.log("current step: ", current_step)

const buttonDict = {
    "scanBtn": "SCAN",
    "connectBtn": "CONNECT",
    // "autoDetectBtn": "AUTO DETECT",
    "disconnectBtn": "DISCONNECT",
    "setDefaultBtn": "SET_DEFAULT",
    "saveBtn": "SAVE",
}

function formatSonarDataReadings(data) {
    if (!data) { return data }
    console.log(data)
    data = JSON.parse(data)["sonar"]["data"]
    data = data.split("\\n")
    data.pop() // remove the " at the end
    data.shift() // remove the " at the start
    
    var container = document.createElement("div")
    for (var i = 0; i < data.length; i++) {
        var div = document.createElement("div")
        div.textContent = data[i]
        container.appendChild(div)
    }
    return container
}

const route_data_formatters = {
    "/sonar_state": (data) => JSON.parse(data)["sonar"]["data"],
    "/sonar_configs": (data) => JSON.parse(data)["sonar"]["data"],
    "/sonar_data": formatSonarDataReadings,
    "/sonar_info": (data) => JSON.parse(data)["sonar"]["data"],
}


console.log("sonar")

// ------------------------------------------------------------------------------------------------
// Functions
function parseStringToInt(str) {
    try {
        return parseInt(str)
    } catch (e) {
        console.log("Parsing of String to Int failed")
        console.log(e)
        throw e
    }
}

function createCmdData(buttonString) {
    return {
        button: buttonString,
    }
}

// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
var gAll_ws_connections = []

function create_ws(ip_addr, route, elementId) {
    const ws = new WebSocket("ws://" + ip_addr + route) // route == /imu_smt != topics -> /imu/configs
    ws.addEventListener('open', function(event) {
        console.log(`${route} socket was opended`)
        ws.send('Hello ws data!');
    });
    ws.onmessage = function(evt) {
        if (route == "/sonar_data") {
            document.getElementById(elementId).replaceChildren(route_data_formatters[route](evt.data))
        } else {
            document.getElementById(elementId).textContent = route_data_formatters[route](evt.data)
        }
        return evt.data
    }
    gAll_ws_connections.push(ws)
}

function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function executeCommand(cmd) {
    console.log(cmd)

    var cmd_dict = {}
    cmd_dict[current_step] = cmd
    cmd_str = JSON.stringify(cmd_dict) // i.e. {imu: {button:__}}
    console.log("send cmd: " + cmd_str)
    var url = "http://" + ip_addr + "/command/" + cmd_str
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    executeCommand(createCmdData(buttonDict[element.id]))    
}

// TODO: create an event s.t. when page changes, clear all web sockets

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
// open web socket connection for /data (imu data) => for the imu readings
create_ws(ip_addr, "/sonar_data", "responseData-data")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /state => for the current state
create_ws(ip_addr, "/sonar_state", "responseData-state")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /info => for user status
create_ws(ip_addr, "/sonar_info", "responseData-info")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /configs => for user status
create_ws(ip_addr, "/sonar_configs", "responseData-configs")

