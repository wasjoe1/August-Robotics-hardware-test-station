// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Set Current step
setCurrentStep()

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

// INIT Sonar
console.log("init sonar...") // log the init-ing component
const gSonars = new Set()

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

function getValueFromSelectComponent(id) {
    const selectElement = document.getElementById(id);
    const selectedValue = selectElement.value;
    if (gSonars.has(selectedValue)) {
        console.error("Unit ID value already exists. Please select another value")
        throw new Error("DuplicateUnitIDError: The selected Unit ID value already exists.");
    }
    return selectedValue
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    try {
        // Check for 1 sonar
        if (gSonars.size > 1) { throw new Error("There is more than 1 sonar connected")}

        // execute srv vall
        const selectedValue = getValueFromSelectComponent("selected-id-value")
        executeSrvCall(formatSonarSrvCallData(
            current_step,
            buttonIdToButtonString[element.id],
            selectedValue))
        
        // if successful, delete value from the set & add new value
        gSonars.clear()
        gSonars.add(selectedValue)
    } catch (e) {
        console.error(e)
        console.log("Setting of Sonar unit id failed")
    }
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

function formatSonarDisplayData(data) {
    var container = undefined
    try {
        data = JSON.parse(data)
    } catch (e) {
        if (e instanceof SyntaxError) {
            console.log(`Data is already a valid JS object: ${data}`)
            return {dataEle: container, dataVal: data} // display data val as is in textcontent
        }
        console.error("An unexpected error occurred in parsing JSON data: " + e.message);
        throw e
    }

    // for object/ array types, display each "prop: val" in block style
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

function formatAndDisplaySonarBoxData(dataVal) {
    // ASSUMPTION: User is instructed to plug in the sonars and not plug in/out additional sonars per use
    for (const id_string in dataVal) {
        // check if its in the set, else add
        if (!gSonars.has(id_string)) { gSonars.add(id_string) }

        // display on html
        sonarBox = document.getElementById(`${id_string}-value`)
        sonarBox.textContent = dataVal[id_string]
    }
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataVal} = formatSonarDisplayData(compData)

    switch(topic) {
        // case "/sonar/topic_data_checker": there's no data checker topic
        case "/sonar/data":
            formatAndDisplaySonarBoxData(dataVal)
            break
        case "/sonar/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData // sonar data is already a string
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}