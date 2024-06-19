// NOTE:
// 1. Anything updated here might cause errors in other _component_.js files,
// please ensure changes are update in all _component_.js files
// 2. Refer to test_example_new_js_file.js when creating new _component_.js files

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------ 
// import step_to_text_dict from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'
// window.onClickComponentPageBtn = onClickComponentPageBtn;
// window.switch_lang = switch_lang;
// window.setCurrentStep = setCurrentStep

// var ws_json
// var hostname
// var download_data
// var is_gs
// var url = window.location.href // this is to get the frontend's current url
const ip_addr = document.location.hostname // this is the backend's ip addr => for GET/ POST reqs
const regex = "http://(.*)/step/(.*)"
var current_step = "home"
var cur_lang = 0 // EN is 0, CN is 1
// const step_to_text_dict = ... // from lang.js

console.log("init index...")
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// functions
// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
function get_element_by_id(id) {
    return document.getElementById(id)
}

function parseStringToInt(str) {
    try {
        return parseInt(str)
    } catch (e) {
        console.log("Parsing of String to Int failed")
        console.log(e)
        throw e
    }
}

function setCurrentStep() {
    var url = window.location.href
    var gFound = url.match(regex)
    current_step = gFound[2]
    cur_lang = 0
    console.log("current step: ", current_step)
    console.log("current language: ", cur_lang == 0 ? "EN" : "CN")
    refresh_page_once(cur_lang)
}

function redirectToPage(page) {
    // Stop the data stream(?) => for now no srv call for disconnect, data streams until physical disconnect
    // Redirect to page
    window.location.href = page; // can be /, /step/imu, /step/inclinometer
}

// TODO: callbacks when user reloads the page & curr lang need to == server lang
// function update_lang() {
//     console.log("updating lang")
//     console.log(server_lang)
//     console.log(cur_lang)
//     if (server_lang != cur_lang) {
//         refresh_page_once(server_lang)
//         cur_lang = server_lang
//     }
// }

// function get_lang_request_cb(response) {
//     console.log(response)
//         // server_lang = Integer.parseInt([response])
//     server_lang = Number(response.substr(1, 1))
//     console.log(typeof server_lang)
//         // console.log(Number(response))
//     console.log(server_lang)
//     update_lang()
//         // refresh_page_once(cur_lang)
// }

// function get_lang_requeset() {
//     console.log("send get_lang")
//     var url = "http://" + ip_addr + "/get_lang"
//     var request = new XMLHttpRequest()
//     request.onreadystatechange = function() {
//         if (request.readyState === 4) {
//             get_lang_request_cb(request.response);
//         }
//     }
//     request.open("GET", url)
//     request.send()
// }

// TODO: call backs when user switches languages via frontend button
// function updata_user_manual(data) {
//     var user_manual = get_id("user_manual")
//     var b = data

//     const regex = /\\n|\\r\\n|\\n\\r|\\r/g;
//     var a = b.split(",")
//     var s = ''
//     for (let x = 0; x < a.length; x++) {
//         console.log(unescape(a[x]))
//         ele = a[x].replace(regex, '<br>');
//         ele = ele.replace("[", '');
//         ele = ele.replace("]", '');
//         ele = ele.replace('"', '');
//         ele = ele.replace('"', '');
//         ele = ele.replace(/'/g, '');
//         // console.log(decode_utf8(ele))
//         s = s + ele
//     }
//     user_manual.innerHTML = s
// }

// function request_cb(response) {
//     console.log(unescape(response))
//     server_lang = cur_lang
//         // console.log("xml callback")
//     updata_user_manual(response)
// }

function refresh_page_once(cur_lang) {
    console.log("refreshing page...")
    // Change text content of headers
    var element_id_to_text_dict = step_to_text_dict[current_step]
    for (var eleId in element_id_to_text_dict) {
        get_element_by_id(eleId).textContent = element_id_to_text_dict[eleId][cur_lang]
    }

    // Hide/ Unhide eng or chi elements
    var arr = [document.getElementsByClassName("english_element"), document.getElementsByClassName("chinese_element")]
    for (let i = 0; i < arr.length; i++) {
        if (i == cur_lang) {
            for (let ele of arr[cur_lang]) { if (ele.classList.contains("hide")) { ele.classList.remove("hide") } } // unhide the new selected language
        } else {
            console.log(arr[i])
            for (let ele of arr[i]) { if (!ele.classList.contains("hide")) { ele.classList.add("hide") } } // hide the new selected language
        }
    }
    
    console.log("page refreshed")
}

function switch_lang() {
    console.log("switching lang...")
    cur_lang = cur_lang == 0 ? 1 : 0
    //TODO: set up a server & frontend coordination for lang change
    // console.log("send cur_lang :" + cur_lang)
    // var url = "http://" + ip_addr + "/switch_lang/" + cur_lang
    // var request = new XMLHttpRequest()
    // request.onreadystatechange = function() {
    //     if (request.readyState === 4) {
    //         request_cb(request.response);
    //     }
    // }
    // request.open("GET", url)
    // request.send()
    refresh_page_once(cur_lang)
    console.log("lang switched")
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SERVICE CALLS
async function executeSrvCall(formattedData) {
    console.log(formattedData)
    try {
        console.log("Executing service call...");
        const data_str = JSON.stringify(formattedData); // i.e. {imu: {button:__, parameter:__}}
        console.log("send data: " + data_str);
        const url = `http://${ip_addr}/command/${data_str}`;
        
        const response = await fetch(url, { method: 'GET' });
        if (response.ok) {
            const responseData = await response.text();
            console.log("Request successful, check ROS side for service call. Status:", response.status);
        } else if (response.status >= 400 && response.status < 500) {
            console.error("Client error. Status:", response.status);
            throw new Error(`ClientError: Client error occurred. Status: ${response.status}, Response: ${await response.text()}`);
        } else if (response.status >= 500) {
            console.error("Server error. Status:", response.status);
            throw new Error(`ServerError: Server error occurred. Status: ${response.status}, Response: ${await response.text()}`);
        } else {
            console.error("Unexpected response. Status:", response.status);
            throw new Error(`UnexpectedResponseError: Unexpected response. Status: ${response.status}, Response: ${await response.text()}`);
        }
    } catch (e) {
        console.error("Network error occurred:", e);
        throw e
    }
}

function formatSrvCallData(component, data) {
    var formattedData = {}
    formattedData[component] = data
    return formattedData
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
const gComponentToData = {
    "imu": { button: "CONNECT", baudrate: "", }, // For this ver, imu is connect
    "inclinometer": { button: "CONNECT", baudrate: "", },
    "cb": { button: "CONNECT", ID: "", },
    "sonar": { button: "CONNECT", ID: "", },
}

async function onClickComponentPageBtn(element) {
    try {
        // redirect page according to component attribute 
        const component = element.getAttribute("component")
        const pageRef = `/step/${component}`
        redirectToPage(pageRef)
    } catch (e) {
        console.log(`failed to connect to ${element.getAttribute("component")}`)
        console.log(e)
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// WEB SOCKET CREATION & TOPICS
const gAll_ws_connections = []

function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function create_ws(ip_addr, topic, elementId, onMessageFunc) {
    try {
        const ws = new WebSocket("ws://" + ip_addr + topic) // topic == /imu/topic_smt
        ws.addEventListener('open', function(event) {
            console.log(`${topic} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = (evt) => {onMessageFunc(evt, topic, elementId)}
        ws.onerror = (e) => {
            console.log("websocket error ", e)
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${topic}`)
        console.error(e)
    }
}

function retrieveComponentData(component, data) {
    return (!data)
    ? data
    : JSON.parse(data)[component]["data"] //TODO: see if its possible to find any other similar abstractions
}

console.log("init-ed index")
