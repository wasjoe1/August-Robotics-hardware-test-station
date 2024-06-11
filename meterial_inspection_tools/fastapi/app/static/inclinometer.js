// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Defined in index.js:
// ip_addr
// current_step
// regex

// Set Current step
setCurrentStep()

const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

console.log("init inclinometer...")

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Functions
// Defined in index.js:
// function parseStringToInt(str)
// function redirectToPage(page)
// function formatSrvCallData(component, data)
// function executeSrvCall(formattedData)
// function create_ws(ip_addr, topic,  elementId, onMessageFunc) => onMessageFunc(evt, topic, elementId) is executed as such
// function retrieveComponentData(component, data)

function formatInclinometerSrvCallData(component, buttonString, baudrate) {
    var data = {
        button: buttonString,
        baudrate: baudrate,
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    executeSrvCall(formatInclinometerSrvCallData(
            current_step,
            buttonIdToButtonString[element.id],
            element.getAttribute("index")))
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
const socketNameToElementId = {
    "/inclinometer/topic_state": "responseData-state",
    "/inclinometer/topic_data": "responseData-data",
    "/inclinometer/topic_data_checker": "responseData-data_checker",
    "/inclinometer/topic_info": "responseData-info",
    // "/inclinometer/topic_info_chinese": "responseData-info_chinese",
    "/inclinometer/topic_configs": "responseData-configs",
    // "/inclinometer/topic_configs_chinese": "responseData-configs_chinese",
}

function formatInclinometerDisplayData(data) {
    var container = undefined
    try {
        data = JSON.parse(data) // for configs & data, parsing of JSON is required
        // if data is already a string, it will throw a syntax error
        // i.e. JSON.parse("s") => error
        //      JSON.parse('"OK"') => returns 'OK'
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
    const {topic, data, ele} = options // use object destructuring
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataVal} = formatInclinometerDisplayData(compData)

    //TODO
    switch(topic) {
        case "/inclinometer/topic_data_checker": //TODO
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
        case "/inclinometer/topic_data": //TODO
            ele.replaceChildren(dataEle)
            break
        case "/inclinometer/topic_configs": //TODO
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}

console.log("init-ed inclinometer")
