// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Defined in lang.js:
// step_to_text_dict

// Defined in index.js:
// ip_addr
// current_step
// regex
// cur_lang // EN is 0, CN is 1

// buttonIdToButtonString
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
// function retrieveComponentData(component, data)
// function refresh_page_once(l) => takes in cur_lang
// function switch_lang()

function formatImuSrvCallData(component, buttonString, baudrate) {
    var data =  {
        button: buttonString,
        baudrate: baudrate, // dont change to int as data has to be in string
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
async function onClickCommandBtn(element) {
    try {
        await executeSrvCall(formatImuSrvCallData(
                current_step,
                buttonIdToButtonString[element.id], 
                element.getAttribute("baudrate")))
    } catch (e) {
        console.error(e)
        console.log("Setting of IMU baudrate failed")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/imu/topic_state": "responseData-state",
    "/imu/topic_data": "responseData-data",
    "/imu/topic_data_checker": "responseData-data_checker",
    "/imu/topic_info": "responseData-info",
    "/imu/topic_info_chinese": "responseData-info_chinese",
    "/imu/topic_configs": "responseData-configs",
    "/imu/topic_configs_chinese": "responseData-configs_chinese",
}

function formatImuDisplayData(data) {
    // split the return char
    data = data.split("\\n")
    
    // remove the " char
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
    return {dataEle: container, dataArr : data}
}

// function create_ws(ip_addr, topic, onMessageFunc) created in index.js
// function onMessageFunc(evt) needs to take in (evt) arg
function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    const compData = retrieveComponentData(current_step, data)
    const {dataEle, dataArr} = formatImuDisplayData(compData) // contains element & dataArr

    switch(topic) {
        case "/imu/topic_data_checker":
            console.log(dataArr[0]) // TEST
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
        case "/imu/topic_data":
        case "/imu/topic_configs_chinese":
        case "/imu/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    // displayDataOnElement(topic=topic, data=data, ele=ele) // JS Doesnt support named parameters!!
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// INIT
window.addEventListener('load', async function() {
    try {
        console.log("windows on load...")
        console.log("init imu...")
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

        console.log("init-ed imu")
    } catch (e) {
        console.log(`failed to connect to ${element.getAttribute("component")}`)
        console.log(e)
    }
});
