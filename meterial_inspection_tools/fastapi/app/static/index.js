// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var url = window.location.href
const regex = "http://(.*)/step/(.*)"

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

function redirectToPage(page) {
    // Stop the data stream(?) => for now no srv call for disconnect, data streams until physical disconnect
    // Redirect to page
    window.location.href = page; // can be /, /step/imu, /step/inclinometer
}

function executeSrvCall(data) {
    data_str = JSON.stringify(data) // i.e. {imu: {button:__, parameter:__}}
    console.log("send data: " + data_str)
    var url = "http://" + ip_addr + "/command/" + data_str
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

function createSrvCallData(component, data) {
    formattedData = {}
    formattedData[component] = data
    return formattedData
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
const gComponentToData = {
    "imu": { button: "AUTO_DETECT", baudrate: "", }, // TODO: come up with a better system for this, for now its AUTO_DETECT, v2.0 will be CONNECT, but need to have a non hard coded system
    "inclinometer": { button: "AUTO_DETECT", baudrate: "", },
}

function onClickComponentPageBtn(element) {
    try {
        // execute the service call 
        const component = element.getAttribute("component")
        const data = gComponentToData[component]
        executeSrvCall(createSrvCallData(component, data))
        
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
// Web socket creation
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
