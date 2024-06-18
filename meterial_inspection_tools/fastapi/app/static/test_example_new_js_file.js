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

// TODO: Set Current step
setCurrentStep()

//TODO
// buttonIdToButtonString
const buttonIdToButtonString = {
    // "_btnId_": "_btn_strin_param_",
    // i.e. "setDefaultBtn": "SET_DEFAULT",
}

//TODO
console.log("init _component_...") // log the init-ing component

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
    const formattedData = formatImuDisplayData(data) // contains element & dataArr //TODO
    const {dataEle, dataArr} = formattedData

    //TODO
    switch(topic) { 
        case "/imu/topic_data_checker":
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
        case "/imu/topic_data": //TODO
            // ele.replaceChildren(dataEle)
            break
        case "/imu/topic_configs": //TODO
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