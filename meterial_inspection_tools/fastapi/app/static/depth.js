// import { lang } from './lang.js'
// import { refresh_page_once_list } from './refresh_once.js'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { setCurrentStepAndLang, executeSrvCall, formatSrvCallData, retrieveComponentData, onClickComponentPageBtn, executeSrvCallToConnect, openWebsockets } from "./indexcopy.js"

// Defined in index.js:
// ip_addr
// current_step
// regex

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

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// 3D Rendering
var gData = undefined
const dataElement = document.getElementById("responseData-data")
var geometry = undefined
var points = undefined
const material = new THREE.PointsMaterial( {size: 0.1, vertexColors: true} )
const scene = new THREE.Scene()
const gridHelper = new THREE.GridHelper()
gridHelper.rotation.x = Math.PI / 2; // Rotate 90 degrees around the X-axis, places the grid on the XY plane
scene.add(gridHelper)

const camera = new THREE.PerspectiveCamera(75, dataElement.clientWidth / dataElement.clientHeight, 0.1, 100) // const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100)
camera.position.z = 2

const renderer = new THREE.WebGLRenderer()
renderer.setSize(dataElement.clientWidth, dataElement.clientHeight) // renderer.setSize(window.innerWidth, window.innerHeight)
dataElement.appendChild(renderer.domElement)

const info = document.createElement('div')
info.style.cssText = 'position:absolute;bottom:10px;left:10px;color:white;font-family:monospace;font-size: 17px;filter: drop-shadow(1px 1px 1px #000000);'
dataElement.appendChild(info)

const controls = new OrbitControls(camera, renderer.domElement)

window.addEventListener('resize', () => {
    // camera.aspect = window.innerWidth / window.innerHeight
    camera.aspect = dataElement.clientWidth / dataElement.clientHeight
    camera.updateProjectionMatrix()
    renderer.setSize(dataElement.clientWidth, dataElement.clientHeight)
})

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

function pointCloud2ToVerticesAndColorsArray(pointCloud2Data) {
    // points config logic
    console.log("configuring points...") // TEST
    var vertices = [] // ~200k points
    var colors = []
    console.log("TEST")
    console.log(pointCloud2Data.coords) // TEST
    console.log(pointCloud2Data["coords"]) // TEST
    for (let i = 0; i < pointCloud2Data.coords.length; i++) {
        var point = pointCloud2Data.coords[i]
        var color = pointCloud2Data.colors[i]
        const x = point[0], y = point[1], z = point[2]
        vertices.push(x, y, z)

        const r = color[0] / 255, g = color[1] / 255, b = color[2] / 255
        colors.push(r, g, b)
    }
    console.log("points configured") // TEST
    return { vertices: vertices, colors: colors }
}

function render3dData() { // INIT function for rendering 3D model
    // Retrieving data from evt
    console.log("Render data is called")
    console.log("getting data from /depth/data ...") // TEST
    var pointCloud2Data = gData
    const { vertices, colors } = pointCloud2ToVerticesAndColorsArray(pointCloud2Data)
    
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
// SOCKET CONFIGS
const socketNameToElementId = {
    "/depth/topic_state": "responseData-state",
    "/depth/topic_data": "responseData-data",
    "/depth/topic_info": "responseData-info",
    "/depth/topic_info_chinese": "responseData-info_chinese",
    // "/depth/topic_data_checker": "responseData-data_checker",
    // "/depth/topic_configs": "responseData-configs",
    // "/depth/topic_configs_chinese": "responseData-configs_chinese",
}

function formatDepthDisplayData(data) {
    var container = undefined
    try {
        data = JSON.parse(data) // for configs & data, parsing of JSON is required
        // if data is already a string, it will throw a syntax error
        // i.e. JSON.parse("s") => error
        //      JSON.parse('"OK"') => returns 'OK'
    } catch (e) {
        if (e instanceof SyntaxError) {
            console.log(`data is already a valid JS object: ${data}`)
            return {dataEle: container, dataVal: data}
        }
        console.error("An unexpected error occurred in parsing JSON data: " + e.message);
        throw e
    }

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
    const {topic, data, ele} = options // use object destructuring
    const compData = retrieveComponentData("depth", data)
    const {dataEle, dataVal} = formatDepthDisplayData(compData)
    console.log("incoming data...")
    console.log(topic)

    //TODO
    switch(topic) {
        case "/depth/topic_data": //TODO
            // Update gData whenever new data is sent
            console.log("Data at '/depth/topic_data'")
            gData = dataVal // returns coords
            break
        case "/depth/topic_configs_chinese":
        case "/depth/topic_configs":
            ele.replaceChildren(dataEle)
            break
        default:
            ele.textContent = compData
    }
}

function onMessageFunc(evt, topic, elementId) { // data is contained in evt.data
    displayDataOnElement({topic:topic, data:evt.data, ele:document.getElementById(elementId)})
    return evt.data
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
// INIT
window.addEventListener('load', async function() {
    try {
        console.log("windows on load...")
        console.log("init depth camera...")
        
        // Set Current step and language
        setCurrentStepAndLang()
        
        // execute service call to connect
        await executeSrvCallToConnect("depth")

        // open websockets
        openWebsockets(socketNameToElementId, onMessageFunc)

        // execute render every 3 secs
        setInterval(() => { if (gData) {render3dData()} }, 3000); // TODO what if data[0] is being replaced when startRenderData is called concurrently??

        console.log("init-ed depth camera")
    } catch (e) {
        console.log(`failed to connect to depth camera`)
        console.log(e)
    }
});

window.onClickComponentPageBtn = onClickComponentPageBtn