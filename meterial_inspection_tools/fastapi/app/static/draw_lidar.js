var canvas 
var canvasWidth 
var canvasHeight 
var ctx 
var canvasData 

function find_element(){
    canvas = document.getElementById("lidar");
    canvasWidth = canvas.width;
    canvasHeight = canvas.height;
    ctx = canvas.getContext("2d");
    canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);   
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

function drawLidar(data) {
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);  
    data.forEach(element => {
        drawPixel(element[0], element[1])
    });
    updateCanvas()
}