// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// import * as THREE from 'three' // this is to import threejs if we use npm

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
    "disconnectBtn": "DISCONNECT",
    "setDefaultBtn": "SET_DEFAULT",
    "saveBtn": "SAVE",
}

console.log("depth")
console.log("test")

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
    try {
        const ws = new WebSocket("ws://" + ip_addr + route) // route == /depth_smt
        ws.addEventListener('open', function(event) {
            console.log(`${route} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = function(evt) {
            // TEST
            if (route == "/depth/data") {
                // Assuming fieldValue is a UTF-8 encoded string containing the Base64-encoded data
                // console.log("evt:", evt) // is a MessageEvent object
                var utf8String = evt.data; // returns the pointcloud data in string form "{"depth": {"header": {"seq": 35...."
                var pointCloud2Data = JSON.parse(evt.data)
                console.log(pointCloud2Data)
                console.log(pointCloud2Data.depth.data)
                // Decode the Base64-encoded string back to the original bytes
                var decodedString = atob(pointCloud2Data.depth.data);
                // Convert the decoded string to a byte array
                var byteArray = new Uint8Array(decodedString.length);
                for (var i = 0; i < decodedString.length; i++) {
                    byteArray[i] = decodedString.charCodeAt(i);
                }
                console.log(typeof(byteArray))
                console.log(byteArray)
            }

            // document.getElementById(elementId).textContent = evt.data
            return evt.data
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${route}`)
        console.error(e)
    }
}
function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function executeCommand(cmd) {
    var cmd_dict = {}
    cmd_dict[current_step] = cmd
    cmd_str = JSON.stringify(cmd_dict) // i.e. {depth: {button:__, parameter:__}}
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
// open web socket connection for /data (depth data) => for the depth readings
create_ws(ip_addr, "/depth/data", "responseData-data")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /state => for the current state
create_ws(ip_addr, "/depth/state", "responseData-state")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /info => for user status
create_ws(ip_addr, "/depth/info", "responseData-info")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /configs => for user status
create_ws(ip_addr, "/depth/configs", "responseData-configs")