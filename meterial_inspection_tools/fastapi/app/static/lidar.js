// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// ip_addr is set
// current_step is set
// regex is set

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
function formatLidarSrvCallData(component) {
    var data =  {
        button: "GET_LASERSCAN",
        // button: "GET_POINTCLOUD",
        model: "G2",
    }
    data = formatSrvCallData(component, data)
    return data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
async function onClickCommandBtn(element) {
    try {
        executeSrvCall(formatLidarSrvCallData(current_step))
    } catch (e) {
        console.error(e)
        console.log("Failed to retrieve lidar data")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS & CREATION
const socketNameToElementId = {
    "/lidar/topic_state": "responseData-state",
    "/lidar/topic_data": "responseData-data",
    "/lidar/topic_data_laserscan": "responseData-data",
    "/lidar/topic_data_pointcloud": "responseData-data",
    "/lidar/topic_info": "responseData-info",
    "/lidar/topic_info_chinese": "responseData-info_chinese",
    // "/lidar/topic_data_checker": "responseData-data_checker",
    // "/lidar/topic_configs": "responseData-configs",
    // "/lidar/topic_configs_chinese": "responseData-configs_chinese",
}

function formatLidarDisplayData(data) {
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

function displayDataOnElement(options) {
    const {topic, data, ele} = options
    const compData = retrieveComponentData("lidar", data)
    const {dataEle, dataVal} = formatLidarDisplayData(compData)

    //TODO
    switch(topic) {
        case "/lidar/topic_data_laserscan": //TODO
            console.log(data)
            console.log(JSON.parse(data))
            console.log(JSON.parse(data)["lidar"])
            var lidarData = JSON.parse(data)["lidar"]

            // convert_data_to_pointcloud(angle_increment, ranges)
            // find_element()
            // drawLidar(data)
                // drawPixel (x, y, r=255, g=0, b=0, a=266, max_len = 6)
                // function updateCanvas()
            
            convert_data_to_pointcloud(lidarData["angle_increment"], lidarData["ranges"])
            find_element()
            drawLidar()
            resetLidarData() // lidar_data = []
            break
        case "/lidar/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
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
        console.log("init lidar...") // log the init-ing component

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

        console.log("init-ed lidar")
    } catch (e) {
        console.log(`failed to connect to lidar`)
        console.log(e)
    }
});