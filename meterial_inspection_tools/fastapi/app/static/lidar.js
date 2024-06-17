// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// ip_addr is set
// current_step is set
// regex is set

// TODO: Set Current step
setCurrentStep()

//TODO
// buttonIdToButtonString
const buttonIdToButtonString = {
    "setDefaultBtn": "SET_DEFAULT",
}

//TODO
console.log("init lidar...") // log the init-ing component

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// Functions
// Defined in index.js:
// function parseStringToInt(str)
// function redirectToPage(page)
// function formatSrvCallData(component, data)
// function executeSrvCall(formattedData)
// function create_ws(ip_addr, topic,  elementId, onMessageFunc) => onMessageFunc(evt, topic, elementId) is executed as such

//TODO
function formatLidarSrvCallData(component, _more_) { //TODO
    var data =  {
        button: buttonString,
        baudrate: baudrate, // has to be string to be accepted in srv calls
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
async function onClickCommandBtn(element) {
    try {
        executeSrvCall(formatLidarSrvCallData(
            current_step,
            buttonIdToButtonString[element.id],
            element.getAttribute("baudrate"))) // TODO
    } catch (e) {
        console.error(e)
        console.log("Setting of _setting_ failed")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
//TODO
const socketNameToElementId = {
    // "socketName": "elementId" //TODO
    "/lidar/topic_state": "responseData-state",
    "/lidar/topic_data": "responseData-data",
    "/lidar/topic_data_checker": "responseData-data_checker",
    "/lidar/topic_info": "responseData-info",
    // "/lidar/topic_info_chinese": "responseData-info_chinese",
    "/lidar/topic_configs": "responseData-configs",
    // "/lidar/topic_configs_chinese": "responseData-configs_chinese",
}

//TODO
function formatLidarDisplayData(data) { //TODO
    data = retrieveComponentData(current_step, data)
    console.log(data)
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
    const compData = retrieveComponentData(current_step, data) //TODO
    const {dataEle, dataArr} = formatLidarDisplayData(data) // contains element & dataArr //TODO

    //TODO
    switch(topic) { 
        case "/lidar/topic_data_checker":
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
        case "/lidar/topic_data": //TODO
            // ele.replaceChildren(dataEle)
            break
        case "/lidar/topic_configs": //TODO
            // ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)}) //TODO
    return evt.data
}

//TODO
for (const socketName in socketNameToElementId) {
    create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc)
}