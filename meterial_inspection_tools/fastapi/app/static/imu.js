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
    "setDefaultBtn": "SET_DEFAULT",
}

console.log("imu")

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

// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
var gAll_ws_connections = []

function create_ws(ip_addr, topic, elementId) {
    try {
        const ws = new WebSocket("ws://" + ip_addr + topic) // topic == /imu/topic_smt
        ws.addEventListener('open', function(event) {
            console.log(`${topic} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = function(evt) {    
            console.log(evt.data)
            const data = JSON.parse(evt.data)["imu"]["data"] // for any topic, this is the standard formatting
            const ele = document.getElementById(elementId)
            // displayDataOnElement(topic=topic, data=data, ele=ele) // JS Doesnt support named parameters!!
            displayDataOnElement({topic:topic, data:data, ele:ele})
            return data
        }
        ws.onerror = (e) => {
            console.log("websocket error ", e)
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${topic}`)
        console.error(e)
    }
}

function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function createCmdData(buttonString, param) {
    return {
        button: buttonString,
        parameter: parseStringToInt(param),
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
    executeCommand(createCmdData(buttonDict[element.id], this.getAttribute("baudrate"))) //TODO change the cmd to not send gParam
}

// TODO: create an event s.t. when page changes, clear all web sockets

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
const socketNameToElementId = {
    "/imu/topic_state": "responseData-state",
    "/imu/topic_data": "responseData-data",
    "/imu/topic_data_checker": "responseData-data_checker",
    "/imu/topic_info": "responseData-info",
    "/imu/topic_info_chinese": "responseData-info_chinese",
    "/imu/topic_configs": "responseData-configs",
    "/imu/topic_configs_chinese": "responseData-configs_chinese",
}
for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName])
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    switch(topic) {
        case "/imu/topic_data_checker":
            if (data == "G") {
                ele.classList.remove("background-red")
                ele.classList.add("background-green")
                ele.textContent == "G"
            } else {
                ele.classList.remove("background-green")
                ele.classList.add("background-red")
                ele.textContent == "NG"
            }
        
        default: // default is only executed when none of the case matches
            ele.textContent == data
    }
}