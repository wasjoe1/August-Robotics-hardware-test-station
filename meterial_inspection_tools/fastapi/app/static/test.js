var canvas, scene, renderer, camera;  // Standard basic objects for three.js

var pointCloud;  // The object that represents the visible cloud of points.

var MAX_POINTS = 10000;  // Number of points available
var points;  // An array of Vector3, containing all possible points
var spinSpeeds;  // An array of Quarterians, containing point rotations for "spin" animation
var driftSpeeds; // An array of Vector3, contianing point velocities for "drift" animation

var settings = {
    animation: '',
    points: 1000,
    pointSize: 2,
}

var geometry;  // Object of type THREE.Geometry containing the visible points.
var material;  // Object of type THREE.PointCloudMaterial for point color and size.


/* Render function draws the content of the canvas.
 * If animating is true, it also updates the vertex locations
 * for this frame of the animation and calles
 * requestAnimationFrame(render) to arrange for the next frame.
 */
function render() {
    if (settings.animation) {
        if (settings.animation === 'spin') {
            updateForSpin();
        }
        else {
            updateForDrift();
        }
        requestAnimationFrame(render);
    }
    renderer.render(scene,camera);
}

function updateForSpin() {
    for (var i = 0; i < geometry.vertices.length; i++) {
        var v = points[i];
        v.applyQuaternion( spinSpeeds[i] );  // applies a rotation about the y-axis
    }
    geometry.verticesNeedUpdate = true;
}

function updateForDrift() {
    for (var i = 0; i < geometry.vertices.length; i++) {
        var v = points[i];
        v.add( driftSpeeds[i] );
        if (v.length() > 1) {
			    // When outside the sphere, change to a random velocity,
				// and multiply the point by 0.9997 to move it back
				// towards the inside of the sphere.  Using a value
				// close to 1 allows the sphere to get a little
				// fuzzy.
            driftSpeeds[i] = randomVelocity();
			v.multiplyScalar(0.9997);
        }
		else if (Math.random() < 0.001) {
			   // change to a new random velocity, with a small probability
            driftSpeeds[i] = randomVelocity();
		}
    }
    geometry.verticesNeedUpdate = true;
}


function createWorld() {
    scene = new THREE.Scene();
    renderer.setClearColor(0x333333);
    camera = new THREE.PerspectiveCamera(30,canvas.width/canvas.height,1,10);
    camera.position.z = 5;
    points = new Array(MAX_POINTS);
    spinSpeeds = new Array(MAX_POINTS);
    driftSpeeds = new Array(MAX_POINTS);
    var i = 0;
    var yaxis = new THREE.Vector3(0,1,0);
    while (i < MAX_POINTS) {
        var x = 2*Math.random() - 1;
        var y = 2*Math.random() - 1;
        var z = 2*Math.random() - 1;
        if ( x*x + y*y + z*z < 1 ) {  // only use points inside the unit sphere
            var angularSpeed = 0.001 + Math.random()/50;  // angular speed of rotation about the y-axis
            spinSpeeds[i] = new THREE.Quaternion();
            spinSpeeds[i].setFromAxisAngle(yaxis,angularSpeed);  // The quaternian for rotation by angularSpeed radians about the y-axis.
            driftSpeeds[i] = randomVelocity();
			points[i] = new THREE.Vector3(x,y,z);
			i++;
        }
    }
    geometry = new THREE.Geometry();
    for (i = 0; i < settings.points; i++) {
        geometry.vertices.push(points[i]);
    }
    material = new THREE.PointsMaterial({
            color: "yellow",
            size: settings.pointSize,
            sizeAttenuation: false
        });
    pointCloud = new THREE.Points(geometry,material);
	scene.add(pointCloud);
}


function randomVelocity() {
    var dx = 0.001 + 0.003*Math.random();
    var dy = 0.001 + 0.003*Math.random();
    var dz = 0.001 + 0.003*Math.random();
    if (Math.random() < 0.5) {
        dx = -dx;
    }
    if (Math.random() < 0.5) {
        dy = -dy;
    }
    if (Math.random() < 0.5) {
        dz = -dz;
    }
    return new THREE.Vector3(dx,dy,dz);
}


/* This will be called when the user changes the setting on the
 * animation radio buttons.
 */
function checkAnimation() {
    if (!settings.animation) {
        animating = false;  // Will stop the animation if one is in progress.
    }
    else if ( ! animating ) { // Start animating, if not currently animating.
        animating = true;
        requestAnimationFrame(render);
    }
}

function init() {
    try {
        canvas = document.getElementById("maincanvas");
        renderer = new THREE.WebGLRenderer({
            canvas: canvas,
            antialias: true
        });
    }
    catch (e) {
        document.getElementById("canvas-holder").innerHTML = 
            "<p><b>Sorry, an error occurred:<br>" + e + "</b></p>";
        return;
    }

    var gui = new dat.GUI(/*{ autoPlace: false }*/);
    //document.getElementById('settings').appendChild(gui.domElement);
    var prevAnim = settings.animation;
    gui.add(settings, 'animation', { None: '', Spin: 'spin', Drift: 'drift' })
        .onChange(anim => {
            if (!prevAnim) { // Start animating, if not currently animating.
                requestAnimationFrame(render);
            }
            prevAnim = anim;
        });
    gui.add(settings, 'points', 10, MAX_POINTS)
        .onChange(p => {
            //http://math.hws.edu/graphicsbook/demos/c5/point-cloud.html
            // Change the number of points. I don't know why I had to create
            // a whole new PointCloud object to implement this.  I was not
            // able to get the renderer to recognize a change in the length
            // of the vertex array in the geometry, or to recognize a new
            // geometry object in the original pointCloud object.
            geometry.dispose();
            geometry = new THREE.Geometry();
            for (var i = 0; i < p; i++) {
                geometry.vertices.push(points[i]);
            }
            scene.remove(pointCloud);
            pointCloud = new THREE.Points(geometry, material);
            scene.add(pointCloud);
        
            if (!settings.animation) {
                render();
            }
        });
    gui.add(settings, 'pointSize', .1, 10)
        .onChange(size => {
            // Change the size of the points by modifying the size property of the matrial.
            material.size = size;
            if (!settings.animation) {
                render();
            }
        });

    createWorld();
    render();
}


init();