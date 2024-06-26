var canvas 
var canvasWidth 
var canvasHeight 
var ctx 
var canvasData
var lidar_data = []

function convert_data_to_pointcloud(angle_increment, ranges) {
    start_ang = 0
    console.log(ranges)
    console.log(angle_increment)
    for (let i = 0; i < ranges.length; i++) {
        ang = start_ang + i*angle_increment
        lidar_data.push([ranges[i]*Math.cos(ang), ranges[i]*Math.sin(ang)]);
    }
}

function find_element() {
    canvas = document.getElementById("responseData-data");
    canvasWidth = canvas.width;
    canvasHeight = canvas.height;
    ctx = canvas.getContext("2d");
    ctx.fillStyle = "black";
    console.log(ctx.fillStyle)
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

// That's how you define the value of a pixel
function drawPixel (x, y, r=255, g=0, b=0, a=266, max_len = 6) {
    pix_len = max_len*2/canvasWidth
    // console.log(pix_len)
    x = x + 6
    y = y + 6
    x = parseInt(x/pix_len)
    y = parseInt(y/pix_len)
    var index = (x + y * canvasWidth) * 4;
    // console.log(x,y,index)
    
    canvasData.data[index + 0] = r;
    canvasData.data[index + 1] = g;
    canvasData.data[index + 2] = b;
    canvasData.data[index + 3] = a;
}

// That's how you update the canvas, so that your
// modification are taken in consideration
function updateCanvas() {
    ctx.putImageData(canvasData, 0, 0);
}

function drawLidar() {
    // ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);  
    lidar_data.forEach(element => {
        drawPixel(element[0], element[1]) // process the data & put into canvasData.data
    });
    updateCanvas() // put data into the canvas
}

function resetLidarData() {
    lidar_data = []
}

// export { convert_data_to_pointcloud, find_element, drawPixel, updateCanvas, drawLidar, resetLidarData }