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
async function executeSrvCall(formattedData) {
    try {
        console.log("Executing service call...");
        const data_str = JSON.stringify(formattedData); // i.e. {imu: {button:__, parameter:__}}
        console.log("send data: " + data_str);
        const url = `http://${ip_addr}/command/${data_str}`;
        
        const response = await fetch(url, { method: 'GET' });
        if (response.ok) {
            const responseData = await response.text();
            console.log("Request successful, check ROS side for service call. Status:", response.status);
        } else if (response.status >= 400 && response.status < 500) {
            console.error("Client error. Status:", response.status);
            throw new Error(`ClientError: Client error occurred. Status: ${response.status}, Response: ${await response.text()}`);
        } else if (response.status >= 500) {
            console.error("Server error. Status:", response.status);
            throw new Error(`ServerError: Server error occurred. Status: ${response.status}, Response: ${await response.text()}`);
        } else {
            console.error("Unexpected response. Status:", response.status);
            throw new Error(`UnexpectedResponseError: Unexpected response. Status: ${response.status}, Response: ${await response.text()}`);
        }
    } catch (e) {
        console.error("Network error occurred:", e);
        throw e
    }
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
    "imu": { button: "CONNECT", baudrate: "", }, // For this ver, imu is connect
    // "imu": { button: "CONNECT", baudrate: "", },
    "inclinometer": { button: "CONNECT", baudrate: "", },
    "cb": { button: "CONNECT", ID: "", },
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
