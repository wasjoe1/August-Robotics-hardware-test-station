// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'

import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'


var ws_json
var hostname
var ip_addr = document.location.hostname
var download_data

var is_gs

var url = window.location.href
const regex = "http://(.*)/step/(.*)"
const found = url.match(regex)
var current_step = found[2]
console.log("current step: ", current_step)

var gParam = undefined
var gSelectedComponentElement = undefined

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
var count = 0
const vertices = [] // ~200k points
const colors = []
const gData = []
var geometry = undefined
var points = undefined
const material = new THREE.PointsMaterial( {size: 0.1, vertexColors: true} )

const scene = new THREE.Scene()

const gridHelper = new THREE.GridHelper()
gridHelper.position.y = -0.5
scene.add(gridHelper)

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100)
camera.position.z = 2

const renderer = new THREE.WebGLRenderer()
renderer.setSize(window.innerWidth, window.innerHeight)
document.body.appendChild(renderer.domElement)

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(window.innerWidth, window.innerHeight)
})

const info = document.createElement('div')
info.style.cssText = 'position:absolute;bottom:10px;left:10px;color:white;font-family:monospace;font-size: 17px;filter: drop-shadow(1px 1px 1px #000000);'
document.body.appendChild(info)

const controls = new OrbitControls(camera, renderer.domElement)

function animate() {
  requestAnimationFrame(animate)

  controls.update()

  info.innerText =
    'Polar Angle : ' +
    ((controls.getPolarAngle() / -Math.PI) * 180 + 90).toFixed(2) +
    '°\nAzimuth Angle : ' +
    ((controls.getAzimuthalAngle() / Math.PI) * 180).toFixed(2) +
    '°'

  renderer.render(scene, camera)
}

// ------------------------------------------------------------------------------------------------
// Functions

function createCmdData(buttonString) {
    return {
        button: buttonString,
    }
}

// TODO: create a web socket manager class to hide all these under the hood implementation (connections, create, get, clear)
var gAll_ws_connections = []
function clear_all_ws() {
    for (const ws in gAll_ws_connections) {
        ws.close()
    }
}

function startRenderData() {
    console.log("rendering data is called")
    var evt = gData[0]
    console.log("getting data from /depth/data ...") // TEST
    var eventData = JSON.parse(evt.data)
    var pointCloud2Data = JSON.parse(eventData.depth.data)
    
    console.log("configuring points...") // TEST
    for (let i = 0; i < pointCloud2Data.coords.length; i++) {
        var point = pointCloud2Data.coords[i]
        var color = pointCloud2Data.colors[i]
        const x = point[0], y = point[1], z = point[2]
        vertices.push(x, y, z)
        const r = color[0] / 255, g = color[1] / 255, b = color[2] / 255
        colors.push(r, g, b)
    }
    console.log("points configured") // TEST
    
    console.log("Trying to render points...") // TEST
    // if there was a geom in scene previously
    if (geometry) {
        geometry.dispose()
        scene.remove(points)
    }
    geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3))
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))
    points = new THREE.Points( geometry, material )
    scene.add( points )
    animate()
    console.log("points rendered") // TEST
}

function create_ws(ip_addr, route, elementId) {
    try {
        const ws = new WebSocket("ws://" + ip_addr + route) // route == /depth_smt
        ws.addEventListener('open', function(event) {
            console.log(`${route} socket was opended`)
            ws.send('Hello ws data!');
        });
        ws.onmessage = function(evt) {
            // TEST
            if (route == "/depth/data") { // TEST
                // replace existing data
                gData[0] = evt

                if (count==0) {
                    // first time start async funtion
                    setInterval(startRenderData, 3000); // TODO what if data[0] is being replaced when startRenderData is called concurrently??
                    count++
                }
            }
            return gData[0].data
        }
        gAll_ws_connections.push(ws)
    } catch (e) {
        console.log(`Failed to create web socket for ${route}`)
        console.error(e)
    }
}

function executeCommand(cmd) {
    var cmd_dict = {}
    cmd_dict[current_step] = cmd
    var cmd_str = JSON.stringify(cmd_dict) // i.e. {depth: {button:__, parameter:__}}
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

window.onClickCommandBtn = onClickCommandBtn;