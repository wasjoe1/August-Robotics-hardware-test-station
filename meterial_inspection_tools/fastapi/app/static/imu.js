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
    // "/imu/topic_info_chinese": "responseData-info_chinese",
    "/imu/topic_configs": "responseData-configs",
    // "/imu/topic_configs_chinese": "responseData-configs_chinese",
}

function formatImuData(data) {
    if (!data) { return data }
    data = JSON.parse(data)["imu"]["data"]
    console.log(data)
    data = data.split("\\n")
    console.log(data)
    let charToRemove = '"'
    var regExToRemoveChar = new RegExp(charToRemove, 'g')
    data[0] = data[0].replace(regExToRemoveChar, '') // remove the " at the start
    data[data.length-1] = data[data.length-1].replace(regExToRemoveChar, '') // remove the " at the end
    
    var container = document.createElement("div")
    for (var i = 0; i < data.length; i++) {
        var p = document.createElement("p")
        p.textContent = data[i]
        container.appendChild(p)
    }
    return {element: container, dataArr : data}
}

// function create_ws(ip_addr, topic, onMessageFunc) created in index.js
// function onMessageFunc(evt) needs to take in (evt) arg
function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    const formattedData = formatImuData(data) // contains element & dataArr
    const dataEle = formattedData.element
    const dataArr = formattedData.dataArr

    switch(topic) {
        case "/imu/topic_data_checker":
            if (dataArr[0] == 'OK') {
                console.log("data is OK") // TEST
                console.log(dataArr[0]) // TEST
                ele.classList.remove("background-red")
                ele.classList.add("background-green")
                ele.textContent = "G"
            } else {
                console.log("data is not OK") // TEST
                console.log(dataArr[0]) // TEST
                ele.classList.remove("background-green")
                ele.classList.add("background-red")
                ele.textContent = "NG"
            }
            break
        case "/imu/topic_data":
            ele.replaceChildren(dataEle)
            break
        case "/imu/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = JSON.parse(data)["imu"]["data"]
    }
}

function onMessageFunc(evt, topic, elementId) {
    // displayDataOnElement(topic=topic, data=data, ele=ele) // JS Doesnt support named parameters!!
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}