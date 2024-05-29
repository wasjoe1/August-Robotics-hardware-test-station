// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var url = window.location.href
var gFound = url.match(regex)
current_step = gFound[2]
console.log("current step: ", current_step)

const buttonDict = {
    "setDefaultBtn": "SET_DEFAULT",
}

console.log("init imu...")

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Functions

// function executeSrvCall(data) is defined in index.js
// function parseStringToInt(str) is defined in index.js

function createSrvCallData(component, buttonString, param) {
    data = {}
    data[component] = {
        button: buttonString,
        parameter: parseStringToInt(param),
    }
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn() {
    executeSrvCall(createSrvCallData(
            current_step,
            buttonDict[this.id], 
            this.getAttribute("baudrate")))
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/imu/topic_state": "responseData-state",
    "/imu/topic_data": "responseData-data",
    "/imu/topic_data_checker": "responseData-data_checker",
    "/imu/topic_info": "responseData-info",
    "/imu/topic_info_chinese": "responseData-info_chinese",
    "/imu/topic_configs": "responseData-configs",
    "/imu/topic_configs_chinese": "responseData-configs_chinese",
}

// function create_ws(ip_addr, topic, onMessageFunc) created in index.js
// function onMessageFunc(evt) needs to take in (evt) arg
function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    switch(topic) {
        case "/imu/topic_data_checker":
            console.log(data) // TEST
            if (data == '"OK"') {
                console.log("data is OK") // TEST
                ele.classList.remove("background-red")
                ele.classList.add("background-green")
                ele.textContent == "G"
            } else {
                console.log("data is not OK") // TEST
                ele.classList.remove("background-green")
                ele.classList.add("background-red")
                ele.textContent == "NG"
            }
        
        default: // default is only executed when none of the case matches
            console.log(ele.id) // TEST
            console.log(ele) // TEST
            ele.textContent == data
    }
}

function onMessageFunc(evt, topic, elementId) {
    console.log(evt.data)
    const data = JSON.parse(evt.data)["imu"]["data"] // for any topic, this is the standard formatting
    const ele = document.getElementById(elementId)
    // displayDataOnElement(topic=topic, data=data, ele=ele) // JS Doesnt support named parameters!!
    displayDataOnElement({topic:topic, data:data, ele:ele})
    return data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}