// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

// import * as THREE from 'three'; // this is to import threejs if we use npm
// const THREE = require('three');
import * as THREE from './node_modules/three/build/three.module.js';

var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var url = window.location.href
const regex = "http://(.*)/step/(.*)"
const found = url.match(regex)
current_step = found[2]
console.log("current step: ", current_step)

gParam = undefined
gSelectedComponentElement = undefined

const buttonDict = {
    "scanBtn": "SCAN",
    "connectBtn": "CONNECT",
    "disconnectBtn": "DISCONNECT",
    "setDefaultBtn": "SET_DEFAULT",
    "saveBtn": "SAVE",
}

console.log("depth")
console.log("test")

// ------------------------------------------------------------------------------------------------
// 3D Rendering

// scene set up
const canvas = document.querySelector("#product-demo");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const sizes = {
  width: window.innerWidth,
  height: window.innerHeight,
};

// camera
const camera = new THREE.PerspectiveCamera(75, sizes.width / sizes.height, 0.1, 1000);
camera.position.set(0, 0, 5); // fix angle of our initial pov (remove to show difference)
scene.add(camera);

// lights
const ambientLight = new THREE.AmbientLight(0xffffff, 15);
ambientLight.position.set(0, 0, 0);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(
  0xffffff,
  0.8
);
directionalLight.position.set(4, 0, 3);
scene.add(directionalLight);


// ------------------------------------------------------------------------------------------------
// Functions
function parseStringToInt(str) {
    try {
        return parseInt(str)
    } catch (e) {
        console.log("Parsing of String to Int failed")
        console.log(e)
        throw e
    }
}

function createCmdData(buttonString) {
    return {
        button: buttonString,
    }
}

// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
var gAll_ws_connections = []

function create_ws(ip_addr, route, elementId) {
    try {
        const ws = new WebSocket("ws://" + ip_addr + route) // route == /depth_smt
        ws.addEventListener('open', function(event) {
            console.log(`${route} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = function(evt) {
            // TEST
            if (route == "/depth/data") {
                // var pointCloud2Data = JSON.parse(evt.data) // evt is websocket message event, {depth: {header: {seq: 35....
                // var decodedString = atob(pointCloud2Data.depth.data); // '\x00\x00\x7F...
                // var byteArray = new Uint8Array(decodedString.length);
                // for (var i = 0; i < decodedString.length; i++) {
                //     byteArray[i] = decodedString.charCodeAt(i);
                // }

                var pointCloud2Data = JSON.parse(evt.data)
                const vertices = [] // not sure how many points this will have
                const colors = []
                for (let i = 0; i < pointCloud2Data.coords.length; i++) {
                    var point = pointCloud2Data.coords[i]
                    var color = pointCloud2Data.colors[i]
                    const x = point[0]
                    const y = point[1]
                    const z = point[2]
                    vertices.push(x, y, z)
                    const r = color[0]
                    const g = color[1]
                    const b = color[2]
                    colors.push(r, g, b)
                }
                const geometry = new THREE.BufferGeometry();
                geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3))
                geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))
                const material = new THREE.PointsMaterial( {vertexColors: true} )
                const points = new THREE.Points( geometry, material )
                screen.add( points )

            }

            // document.getElementById(elementId).textContent = evt.data
            return evt.data
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${route}`)
        console.error(e)
    }
}
function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function executeCommand(cmd) {
    var cmd_dict = {}
    cmd_dict[current_step] = cmd
    cmd_str = JSON.stringify(cmd_dict) // i.e. {depth: {button:__, parameter:__}}
    console.log("send cmd: " + cmd_str)
    var url = "http://" + ip_addr + "/command/" + cmd_str
    var request = new XMLHttpRequest()
    request.open("GET", url)
    request.send()
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// onClickEvents
function onClickCommandBtn(element) {
    executeCommand(createCmdData(buttonDict[element.id]))
}

function onClickSetParamBtn(element) {
    gParam = element.getAttribute("parameter")
    console.log(gParam)
    if (gSelectedComponentElement) {
        gSelectedComponentElement.classList.remove("selected")
    }
    element.classList.add("selected")
    gSelectedComponentElement = element
}

// TODO: create an event s.t. when page changes, clear all web sockets

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// SOCKET CONFIGS
// open web socket connection for /data (depth data) => for the depth readings
create_ws(ip_addr, "/depth/data", "responseData-data")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /state => for the current state
create_ws(ip_addr, "/depth/state", "responseData-state")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /info => for user status
create_ws(ip_addr, "/depth/info", "responseData-info")

// ------------------------------------------------------------------------------------------------
// open web socket connection for /configs => for user status
create_ws(ip_addr, "/depth/configs", "responseData-configs")