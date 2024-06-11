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
        baudrate: unitid,
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
    data = retrieveComponentData(current_step, data)
    //TODO
    // data = data.split("\\n") 
    // console.log(data)
    // let charToRemove = '"'
    // var regExToRemoveChar = new RegExp(charToRemove, 'g')
    // data[0] = data[0].replace(regExToRemoveChar, '') // remove the " at the start
    // data[data.length-1] = data[data.length-1].replace(regExToRemoveChar, '') // remove the " at the end

    //TODO
    // var container = document.createElement("div")
    // for (var i = 0; i < data.length; i++) {
    //     var p = document.createElement("p")
    //     p.textContent = data[i]
    //     container.appendChild(p)
    // }

    //TODO
    return {dataEle: container, dataArr : data} //TODO if no ele container == undefined
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    const formattedData = formatCBDisplayData(data) // contains element & dataArr //TODO
    const {dataEle, dataArr} = formattedData

    //TODO
    switch(topic) { 
        case "/cb/topic_data_checker":
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
        case "/cb/topic_data": //TODO
            // ele.replaceChildren(dataEle)
            break
        case "/cb/topic_configs": //TODO
            // ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = retrieveComponentData(current_step, data) //TODO
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