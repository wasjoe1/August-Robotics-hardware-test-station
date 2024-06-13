// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Set Current step
setCurrentStep()

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

//TODO
console.log("init sonar...") // log the init-ing component

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Functions
// Defined in index.js:
// function parseStringToInt(str)
// function redirectToPage(page)
// function formatSrvCallData(component, data)
// function executeSrvCall(formattedData)
// function create_ws(ip_addr, topic,  elementId, onMessageFunc) => onMessageFunc(evt, topic, elementId) is executed as such

function formatSonarSrvCallData(component, buttonString, unitid) {
    var data =  {
        button: buttonString,
        ID: unitid, // is a string
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    executeSrvCall(formatSonarSrvCallData(
            current_step,
            buttonIdToButtonString[element.id],
            element.getAttribute("unitid"))) //TODO: change this attribute that im getting the data from
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/sonar/topic_state": "responseData-state",
    "/sonar/topic_data": "responseData-data",
    // "/sonar/topic_data_checker": "responseData-data_checker", // refer to ros_interface.py, for this ver, sonar does not have a data_checker topic
    "/sonar/topic_info": "responseData-info",
    // "/sonar/topic_info_chinese": "responseData-info_chinese",
    "/sonar/topic_configs": "responseData-configs",
    // "/sonar/topic_configs_chinese": "responseData-configs_chinese",
}

//TODO
function formatSonarDisplayData(data) { //TODO
    var container = undefined
    try {
        data = JSON.parse(data)
    } catch (e) {
        if (e instanceof SyntaxError) {
            console.log(`data is already a valid JS object: ${data}`)
            return {dataEle: container, dataVal: data}
        }
        console.error("An unexpected error occurred in parsing JSON data: " + e.message);
        throw e
    }

    if (typeof(data) == "object" && data != null) {
        container = document.createElement("div")
        for (var prop in data) {
            var p = document.createElement("p")
            p.textContent = `${prop}: ${data[prop]}`
            container.appendChild(p)
        }
    }
    return {dataEle: container, dataVal: data}
}

function displayDataOnElement(options) {//TODO
    const {topic, data, ele} = options
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataVal} = formatSonarDisplayData(compData)

    //TODO
    switch(topic) { 
        // case "/sonar/topic_data_checker": there's no data checker topic
        case "/sonar/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData // sonar data is already a string
    }
}

//TODO currently each onMessageFunc is different for each component, hence not abstracted away
function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)}) //TODO
    return evt.data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}