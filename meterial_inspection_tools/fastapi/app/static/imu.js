// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

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

gParam = undefined
gSelectedComponentElement = undefined

const buttonDict = {
    "scanBtn": "SCAN",
    "connectBtn": "CONNECT",
    // "autoDetectBtn": "AUTO DETECT",
    "disconnectBtn": "DISCONNECT",
    "setDefaultBtn": "SET_DEFAULT",
    "saveBtn": "SAVE",
}

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

function createCmdData(buttonString, param) {
    return {
        button: buttonString,
        parameter: param,
    }
}

// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
var gAll_ws_connections = []

function create_ws(ip_addr, route, elementId) {
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
    cmd_str = JSON.stringify(cmd_dict) // i.e. {imu: {button:__, parameter:__}}
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
    executeCommand(createCmdData(buttonDict[element.id], gParam))    
}

function onClickSetParamBtn(element) {
    gParam = element.getAttribute("parameter")
    console.log(gParam)
    if (gSelectedComponentElement) {
        gSelectedComponentElement.classList.remove("selected")
    }
    element.classList.add("selected")
    gSelectedComponentElement = element
}

// TODO: create an event s.t. when page changes, clear all web sockets

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
// open web socket connection for /data (imu data) => for the imu readings
create_ws(ip_addr, "/imu_data", "responseData-data")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /state => for the current state
create_ws(ip_addr, "/imu_state", "responseData-state")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /info => for user status
create_ws(ip_addr, "/imu_info", "responseData-info")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /configs => for user status
create_ws(ip_addr, "/imu_configs", "responseData-configs")
