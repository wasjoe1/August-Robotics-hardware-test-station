// NOTE:
// 1. Anything updated here might cause errors in other _component_.js files,
// please ensure changes are update in all _component_.js files
// 2. Refer to test_example_new_js_file.js when creating new _component_.js files

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------ 
// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// var ws_json
// var hostname
// var download_data
// var is_gs
// var url = window.location.href // this is to get the frontend's current url
const ip_addr = document.location.hostname // this is the backend's ip addr => for GET/ POST reqs
const regex = "http://(.*)/step/(.*)"
var current_step = undefined

console.log("init index...")
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// functions
// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
function parseStringToInt(str) {
    try {
        return parseInt(str)
    } catch (e) {
        console.log("Parsing of String to Int failed")
        console.log(e)
        throw e
    }
}

function setCurrentStep() {
    var url = window.location.href
    var gFound = url.match(regex)
    current_step = gFound[2]
    console.log("current step: ", current_step)
}

function redirectToPage(page) {
    // Stop the data stream(?) => for now no srv call for disconnect, data streams until physical disconnect
    // Redirect to page
    window.location.href = page; // can be /, /step/imu, /step/inclinometer
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SERVICE CALLS
function executeSrvCall(formattedData) {
    console.log("Executing service call...")
    data_str = JSON.stringify(formattedData) // i.e. {imu: {button:__, parameter:__}}
    console.log("send data: " + data_str)
    var url = "http://" + ip_addr + "/command/" + data_str
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.onload = function () {
        if (request.status >= 200 && request.status < 300) {
            console.log("Request successful, check ROS side for service call. status:", request.responseText);
        } else if (request.status >= 400 && request.status < 500) {
            console.error("Client error. Status:", request.status);
        } else if (request.status >= 500) {
            console.error("Server error. Status:", request.status);
        } else {
            console.error("Unexpected response. Status:", request.status);
        }
    };
    request.onerror = function () {
        console.error("Network error occurred.");
    };
    request.send()
}

function formatSrvCallData(component, data) {
    formattedData = {}
    formattedData[component] = data
    return formattedData
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
const gComponentToData = {
    "imu": { button: "AUTO_DETECT", baudrate: "", }, // For this ver, imu is auto_detect
    // "imu": { button: "CONNECT", baudrate: "", },
    "inclinometer": { button: "CONNECT", baudrate: "", },
}

function onClickComponentPageBtn(element) {
    try {
        // execute the service call
        const component = element.getAttribute("component")
        const initData = gComponentToData[component]
        executeSrvCall(formatSrvCallData(component, initData))
        
        // on successful connection, switch pages
        const pageRef = `/step/${component}`
        redirectToPage(pageRef)
    } catch (e) {
        console.log(`failed to connect to ${element.getAttribute("component")}`)
        console.log(e)
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// WEB SOCKET CREATION & TOPICS
const gAll_ws_connections = []

function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function create_ws(ip_addr, topic, elementId, onMessageFunc) {
    try {
        const ws = new WebSocket("ws://" + ip_addr + topic) // topic == /imu/topic_smt
        ws.addEventListener('open', function(event) {
            console.log(`${topic} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = (evt) => {onMessageFunc(evt, topic, elementId)}
        ws.onerror = (e) => {
            console.log("websocket error ", e)
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${topic}`)
        console.error(e)
    }
}

function retrieveComponentData(component, data) {
    return (!data)
    ? data
    : JSON.parse(data)[component]["data"] //TODO
}

console.log("init-ed index")
