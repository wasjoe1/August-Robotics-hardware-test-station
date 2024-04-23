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

const buttonDict = {
    "scanBtn": "SCAN",
    "saveBtn": "SAVE",
    "connectBtn": "CONNECT",
    "closeBtn": "CLOSE",
    "setBtn": "SET",
}

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
    var inputArr = [0,0,0,0]
    if (buttonString == "SET") { // if not saved, input arr inputs will be 0
        inputArr = [
            parseStringToInt(document.getElementById("imuInput1").value),
            parseStringToInt(document.getElementById("imuInput2").value),
            parseStringToInt(document.getElementById("imuInput3").value),
            parseStringToInt(document.getElementById("imuInput4").value),
        ]
    }

    return {
        button: buttonString,
        parameter1: inputArr[0],
        parameter2: inputArr[1],
        parameter3: inputArr[2],
        parameter4: inputArr[3],
    }
}

function executeCommand(cmd) {
    console.log(cmd)

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

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickBtn(element) {
    executeCommand(createCmdData(buttonDict[element.id]))    
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
