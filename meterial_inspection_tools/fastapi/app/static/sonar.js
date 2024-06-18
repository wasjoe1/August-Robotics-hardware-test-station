// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

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
    if (selectedValue == "false") {
        console.error("Unit ID value is invalid. Please select a valid Unit ID")
        throw new Error("InvalidUnitIDError: The selected Unit ID value is invalid.");
    }
    return selectedValue
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
async function onClickCommandBtn(element) {
    try {
        // Check for 1 sonar
        if (gSonars.size > 1) { throw new Error("There is more than 1 sonar connected")}

        // execute srv vall
        const selectedValue = getValueFromSelectComponent("selected-id-value")
        await executeSrvCall(formatSonarSrvCallData(
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
    // ASSUMPTION: data is in this format: {"0xfa": "0.06" , "_unit_id_": "_value_"}
    console.log(dataVal) // TEST: await for ROS test completion then del this
    for (const id_string in dataVal) {
        // check if its in the set, else add
        if (!gSonars.has(id_string)) { gSonars.add(id_string) }

        // display on html
        sonarBox = document.getElementById(`${id_string}-value`)
        sonarBox.textContent = dataVal[id_string]
        sonarBox.parentNode.classList.remove("background-red")
    }

    // check number of sonars 
    var set_id_container = document.getElementById("set-unit-id-container")
    if (gSonars.size == 1) {
        document.getElementById("current-unit-id").value = [...gSonars][0]
        if (set_id_container.classList.contains("hide")) { set_id_container.classList.remove("hide") }
    } else {
        if (!set_id_container.classList.contains("hide")) { set_id_container.classList.add("hide") }
    }
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataVal} = formatSonarDisplayData(compData)

    switch(topic) {
        // case "/sonar/topic_data_checker": there's no data checker topic
        case "/sonar/topic_data":
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

// INIT
window.addEventListener('load', async function() {
    try {
        console.log("windows on load...")
        console.log("init sonar...") // log the init-ing component

        // Reset the set
        gSonars = new Set()

        // Set Current step
        setCurrentStep()
        
        // Refresh the page (language setting)
        refresh_page_once(cur_lang)
    
        // execute the service call
        const initData = gComponentToData[current_step]
        await executeSrvCall(formatSrvCallData(current_step, initData))

        // open websockets
        for (const socketName in socketNameToElementId) {
            create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
        }

        console.log("init-ed sonar")
    } catch (e) {
        console.log(`failed to connect to ${element.getAttribute("component")}`)
        console.log(e)
    }
});
