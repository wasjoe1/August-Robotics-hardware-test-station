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
        baudrate: parseStringToInt(baudrate),
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn() {
    executeSrvCall(formatInclinometerSrvCallData(
            current_step,
            buttonIdToButtonString[this.id],
            this.getAttribute("baudrate")))
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
    data = retrieveComponentData(current_step, data)
    return {dataEle: undefined, dataArr : data}
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    const formattedData = formatInclinometerDisplayData(data) // contains element & dataArr //TODO
    const {dataEle, dataArr} = formattedData

    //TODO
    switch(topic) { 
        case "/inclinometer/topic_data_checker": //TODO
            if (dataArr[0] == 'OK') {
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
        // case "/inclinometer/topic_data": //TODO
        //     ele.replaceChildren(dataEle)
        //     break
        // case "/inclinometer/topic_configs": //TODO
        //     ele.replaceChildren(dataEle)
        //     break
        default:
            ele.textContent = retrieveComponentData(current_step, data)
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}