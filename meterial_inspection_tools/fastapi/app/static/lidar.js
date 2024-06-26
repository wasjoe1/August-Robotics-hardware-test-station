// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { setCurrentStepAndLang, executeSrvCall, formatSrvCallData, retrieveComponentData, onClickComponentPageBtn, executeSrvCallToConnect, openWebsockets } from "./indexcopy.js"


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
        executeSrvCall(formatLidarSrvCallData("lidar"))
    } catch (e) {
        console.error(e)
        console.log("Failed to retrieve lidar data")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// 3D Rendering

// document // document.body.appendChild(renderer.domElement)
//     -> info
//     -> controls // const controls = new OrbitControls(camera, renderer.domElement)
//         -> renderer // renderer.render(scene, camera)
//             -> camera
//             -> scene
//                 -> grid helper // scene.add(gridHelper) 
//                 -> points // scene.add( points ) // points = new THREE.Points( geometry, material )
//                     -> material
//                     -> geometry
//                         -> vertices // geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3))
//                         -> colors // geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))
const dataElement = document.getElementById("responseData-data")
var geometry = undefined
var points = undefined
const material = new THREE.PointsMaterial( {size: 0.1, vertexColors: true} )
const scene = new THREE.Scene()
const gridHelper = new THREE.GridHelper()
gridHelper.rotation.x = Math.PI / 2; // Rotate 90 degrees around the X-axis, places the grid on the XY plane
scene.add(gridHelper)

// const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100)
const camera = new THREE.PerspectiveCamera(75, dataElement.clientWidth / dataElement.clientHeight, 0.1, 100)
camera.position.z = 2

const renderer = new THREE.WebGLRenderer()
// renderer.setSize(window.innerWidth, window.innerHeight)
renderer.setSize(dataElement.clientWidth, dataElement.clientHeight)
dataElement.appendChild(renderer.domElement)

window.addEventListener('resize', () => {
    // camera.aspect = window.innerWidth / window.innerHeight
    camera.aspect = dataElement.clientWidth / dataElement.clientHeight
    camera.updateProjectionMatrix()
    console.log(dataElement.clientWidth)
    console.log(dataElement.clientHeight)
    renderer.setSize(dataElement.clientWidth, dataElement.clientHeight)
})

const info = document.createElement('div')
info.style.cssText = 'position:absolute;bottom:10px;left:10px;color:white;font-family:monospace;font-size: 17px;filter: drop-shadow(1px 1px 1px #000000);'
dataElement.appendChild(info)

const controls = new OrbitControls(camera, renderer.domElement)

function animate() {
    // everytime points change, animate is called
    requestAnimationFrame(animate)

    controls.update()

    info.innerText =
            'Polar Angle : ' +
            ((controls.getPolarAngle() / -Math.PI) * 180 + 90).toFixed(2) +
            '°\nAzimuth Angle : ' +
            ((controls.getAzimuthalAngle() / Math.PI) * 180).toFixed(2) +
            '°'

    renderer.render(scene, camera) // renderer will render a new scene & camera
}

function convert_laserscan_to_vertices_and_colors(angle_increment, ranges, intensities) {
    var vertices = []
    var colors = []
    console.log("configuring points...") // TEST
    var start_ang = 0
    console.log(ranges) // TEST
    console.log(angle_increment) // TEST
    for (let i = 0; i < ranges.length; i++) {
        // Vertices
        var ang = start_ang + i*angle_increment
        vertices.push(ranges[i]*Math.cos(ang), ranges[i]*Math.sin(ang), 0); // y is always set to 0 for now

        // Colors
        // TODO: figure out if the lidar intensity values are really <255
        const r = intensities[i] / 255, g = 1, b = 1
        colors.push(r,g,b)
    }
    console.log("points configured") // TEST
    return { vertices: vertices, colors: colors }
}

function render3dData(angle_increment, ranges, intensities) { // INIT function for rendering 3D model
    // Retrieving data from evt
    console.log("Render data is called")
    console.log("getting data from /depth/data ...") // TEST
    const { vertices, colors } = convert_laserscan_to_vertices_and_colors(angle_increment, ranges, intensities)
    
    console.log("Rendering points...") // TEST
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
    animate() // everytime points change, animate is called
    console.log("points rendered") // TEST
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
    console.log("data is sent")
    console.log(topic)

    //TODO
    switch(topic) {
        case "/lidar/topic_data_laserscan": //TODO
            console.log(data)
            console.log(JSON.parse(data))
            console.log(JSON.parse(data)["lidar"])
            var lidarData = JSON.parse(data)["lidar"]
            render3dData(lidarData["angle_increment"], lidarData["ranges"], lidarData["intensities"])
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

        // Set Current step and language
        setCurrentStepAndLang()
    
        // execute service call to connect
        await executeSrvCallToConnect("lidar")

        // open websockets
        openWebsockets(socketNameToElementId, onMessageFunc)

        // call data in intervals
        setInterval(() => onClickCommandBtn("lidar"), 3000)

        console.log("init-ed lidar")
    } catch (e) {
        console.log(`failed to connect to lidar`)
        console.log(e)
    }
});

window.onClickCommandBtn = onClickCommandBtn
window.onClickComponentPageBtn = onClickComponentPageBtn