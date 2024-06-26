// INSTRUCTIONS
// Add index.js to newly created html page
// add in respective page & command buttons

// Steps in creating js page
// 1. set current step
// 2. Create buttonIdToButtonString dictionary
// 3. create format_component_SrvCallData(component, ....) => use formatSrvCallData(component, data)
// 4. create onClickCommandButton() => use executeSrvCall(formattedData) & format_component_SrvCallData(component, ....)
// 5. open websockets
  // - create socketNameToElementId dictionary
  // - create format_component_DisplayData(data) => use formatSrvCallData
  // - create displayDataOnElement(options) => use format_component_DisplayData(data), options == {topic, data, element}
  // - create onMessageFunc(evt, topic, elementId) => use displayDataOnElement(options), data == evt.data
  // - open websockets using a for loop & create_ws(ip_addr, socketName, socketNameToElementId[socketName], onMessageFunc) => use onMessageFunc & socketNameToElementId dictionary

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// EXAMPLE JS PAGE, copy from here

// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// Defined in lang.js:
// step_to_text_dict

// ip_addr is set
// current_step is set
// regex is set
// cur_lang // EN is 0, CN is 1

//TODO
// buttonIdToButtonString
const buttonIdToButtonString = {
    // "_btnId_": "_btn_strin_param_",
    // i.e. "setDefaultBtn": "SET_DEFAULT",
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
// function refresh_page_once(l) => takes in cur_lang
// function switch_lang()

//TODO
function format_component_SrvCallData(component, _more_) { //TODO
    var data =  {
        // wtv you want to include in the srv call, i.e.
        // button: buttonString,
        // baudrate: baudrate, // has to be string to be accepted in srv calls
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
async function onClickCommandBtn(element) {
    try {
        executeSrvCall(formatSonarSrvCallData( //TODO
            current_step,
            // buttonIdToButtonString[element.id], //TODO 
            // element.getAttribute("baudrate"))) //TODO: change this attribute that im getting the data from
    ))
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
    // "/imu/topic_state": "responseData-state",
    // "/imu/topic_data": "responseData-data",
    // "/imu/topic_data_checker": "responseData-data_checker",
    // "/imu/topic_info": "responseData-info",
    // "/imu/topic_info_chinese": "responseData-info_chinese",
    // "/imu/topic_configs": "responseData-configs",
    // "/imu/topic_configs_chinese": "responseData-configs_chinese",
}

//TODO
function formatImuDisplayData(data) { //TODO
    // TODO
    // var container = undefined
    // try {
    //     data = JSON.parse(data)
    // } catch (e) {
    //     if (e instanceof SyntaxError) {
    //         console.log(`Data is already a valid JS object: ${data}`)
    //         return {dataEle: container, dataVal: data} // display data val as is in textcontent
    //     }
    //     console.error("An unexpected error occurred in parsing JSON data: " + e.message);
    //     throw e
    // }

    // TODO
    // // for object/ array types, display each "prop: val" in block style
    // if (typeof(data) == "object" && data != null) {
    //     container = document.createElement("div")
    //     for (var prop in data) {
    //         var p = document.createElement("p")
    //         p.textContent = `${prop}: ${data[prop]}`
    //         container.appendChild(p)
    //     }
    // }

    //TODO
    return {dataEle: container, dataVal : data} //TODO if no ele container == undefined
}

function displayDataOnElement(options) {
    const {topic, data, ele} = options // use object destructuring
    const compData = retrieveComponentData(current_step, data) //TODO
    const {dataEle, dataVal} = formatImuDisplayData(compData) // contains element & dataArr //TODO

    //TODO
    switch(topic) { 
        case "/inclinometer/topic_data_checker": //TODO
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
        case "/imu/topic_data": //TODO
            // ele.replaceChildren(dataEle)
            break
        case "/imu/topic_configs": //TODO
            // ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
    }
}

//TODO currently each onMessageFunc is different for each component, hence not abstracted away
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
        console.log("init _component_...") // log the init-ing component

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

        console.log("init-ed _component_")
    } catch (e) {
        console.log(`failed to connect to _component_`)
        console.log(e)
    }
});