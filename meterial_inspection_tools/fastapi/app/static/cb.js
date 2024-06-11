// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Set Current step
setCurrentStep()

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

//TODO
console.log("init cb...") // log the init-ing component

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Functions
// Defined in index.js:
// function parseStringToInt(str)
// function redirectToPage(page)
// function formatSrvCallData(component, data)
// function executeSrvCall(formattedData)
// function create_ws(ip_addr, topic,  elementId, onMessageFunc) => onMessageFunc(evt, topic, elementId) is executed as such

function formatCBSrvCallData(component, buttonString, unitid) {
    var data =  {
        button: buttonString,
        ID: unitid,
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    executeSrvCall(formatCBSrvCallData(
            current_step,
            buttonIdToButtonString[element.id],
            element.getAttribute("unitid")))
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/cb/topic_state": "responseData-state",
    "/cb/topic_data": "responseData-data",
    // "/cb/topic_data_checker": "responseData-data_checker", // refer to ros_interface.py, for this ver, cb does not have a data_checker topic
    "/cb/topic_info": "responseData-info",
    // "/cb/topic_info_chinese": "responseData-info_chinese",
    "/cb/topic_configs": "responseData-configs",
    // "/cb/topic_configs_chinese": "responseData-configs_chinese",
}

//TODO
function formatCBDisplayData(data) { //TODO
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

function displayDataOnElement(options) {
    const {topic, data, ele} = options
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataVal} = formatCBDisplayData(compData)

    //TODO
    switch(topic) { 
        // case "/cb/topic_data_checker": there's no data checker topic
        case "/cb/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData // CB data is already a string
    }
}

//TODO currently each onMessageFunc is different for each component, hence not abstracted away
function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)}) //TODO
    return evt.data
}

//TODO
for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}