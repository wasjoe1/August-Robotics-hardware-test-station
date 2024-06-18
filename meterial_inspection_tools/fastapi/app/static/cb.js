// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

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
async function onClickCommandBtn(element) {
    try {
        await executeSrvCall(formatCBSrvCallData(
                current_step,
                buttonIdToButtonString[element.id],
                element.getAttribute("unitid")))
    } catch (e) {
        console.error(e)
        console.log("Setting of CB unit id failed")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/cb/topic_state": "responseData-state",
    "/cb/topic_data": "responseData-data",
    "/cb/topic_data_checker": "responseData-data_checker",
    "/cb/topic_info": "responseData-info",
    "/cb/topic_info_chinese": "responseData-info_chinese",
    "/cb/topic_configs": "responseData-configs",
    "/cb/topic_configs_chinese": "responseData-configs_chinese",
}

function formatCBDisplayData(data) {
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
        case "/cb/topic_data_checker":
            console.log(dataVal)
            if (dataVal == 'OK') {
                console.log("data is OK") // TEST
                ele.classList.remove("background-red")
                ele.classList.add("background-green")
                ele.textContent = "G"
            } else {
                console.log("data is not OK") // TEST
                ele.classList.remove("background-green")
                ele.classList.add("background-red")
                ele.textContent = "NG"
            }
            break
        case "/cb/topic_configs_chinese":
        case "/cb/topic_configs":
            var data_checker_container = document.getElementById("responseData-data_checker-container")
            if (dataVal["model"] == "BRITER") {
                console.log(dataVal["model"])
                if (data_checker_container.classList.contains("hide")) { ele.classList.remove("hide") } // unhide the NG/ G
            } else {
                console.log(dataVal["model"])
                if (!data_checker_container.classList.contains("hide")) { ele.classList.add("hide") } // hide the NG/ G
            }
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData // CB data is already a string
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)}) //TODO
    return evt.data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// INIT
window.addEventListener('load', async function() {
    try {
        console.log("windows on load...")
        console.log("init cb...") // log the init-ing component
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

        console.log("init-ed cb")
    } catch (e) {
        console.log(`failed to connect to ${element.getAttribute("component")}`)
        console.log(e)
    }
});
