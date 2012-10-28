var url = "http://3xrp.localtunnel.com/android",
    hokuyoFilename = "hokuyo.txt";

var world_origin_x = CANVAS_WIDTH/2, 
    world_origin_y = CANVAS_HEIGHT/2,
    world_width = 15, 
    world_height = 20;

var CANVAS_WIDTH, CANVAS_HEIGHT;

window.onload = startup;

function startup() {
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");
    
    CANVAS_WIDTH = canvas.width;
    CANVAS_HEIGHT = canvas.height;

    context.setTransform(CANVAS_WIDTH/world_width, 0, 
                        0, -CANVAS_HEIGHT/world_height, 
                        CANVAS_WIDTH/2, CANVAS_HEIGHT/2);

    setInterval("getFile('"+hokuyoFilename+"', pointsFileHandler, false)", 100);
}

//
// LIDAR display functions
//

function pointsFileHandler(filetext) {
    //console.log("updating points!");

    var lines = filetext.split("\n");
    var ranges = lines[0].split(" "),
        minrange = parseFloat(ranges[0]),
        maxrange = parseFloat(ranges[1]);

    if (lines.length-2 <= 0) {
        console.log("[",filetext,"]");
    }

    var points = new Array(lines.length-2);
    for (var i = 1; i < lines.length-1; i++) {
        var line = lines[i];
        var tokens = line.split(" ");     
        points[i-1] = [parseFloat(tokens[0]), parseFloat(tokens[1])];
    }

    drawPoints(points);
}

function drawPoints(points) {
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");

    context.fillStyle = "lightGray";
    context.fillRect(-world_width/2, -world_height/2, world_width, world_height);
    
    context.lineWidth = .1;
    context.strokeStyle = "black";
    context.fillStyle = "black";
    //context.beginPath();
    for (var i = 0; i < points.length; i++) {
        context.beginPath();
        context.arc(points[i][0], points[i][1], .05, 0, 2*Math.PI, true);
        context.fill();
        //context.lineTo(points[i][0], points[i][1]);
    }
    //context.stroke();

    context.lineWidth = .05;
    context.strokeStyle = "blue";
    context.fillStyle = "blue";
    drawOrigin(context);
}   

function drawOrigin(context) {
    context.beginPath();
    context.moveTo(0,0);
    context.lineTo(0, 1);
    context.lineTo(.2, .8);
    context.stroke();

    context.beginPath();
    context.moveTo(0, 1);    
    context.lineTo(-.2, .8);
    context.stroke();

    context.beginPath();
    context.arc(0, 0, .1, 0, 2*Math.PI, true);
    context.fill();
}

//
// Command/Control functions
//

var keyToDirMap = {};
keyToDirMap["W".charCodeAt(0)] = "forward";
keyToDirMap["S".charCodeAt(0)] = "backward";
keyToDirMap["A".charCodeAt(0)] = "left";
keyToDirMap["D".charCodeAt(0)] = "right";

var dirToValsMap = {
    "forward" :     {x:    0, y:  127},
    "backward" :    {x:    0, y: -127},
    "left" :        {x: -127, y:    0},
    "right" :       {x:  127, y:    0}
}

function keypressed(event) {
    console.log("received key press!");

    var key = event.which;
    sendCommand(keyToDirMap[key]);
}

function sendCommand(dir) {
    console.log("received command:",dir);

    var vals = dirToValsMap[dir];

    if (vals) {
        getFile(url+"?x="+vals.x+"&y="+vals.y, false, true);
    }
}

//
// Ajax functions
//

function getFile(filename, recfunct, isAsync) {

	var ajaxRequest;

	if(window.XMLHttpRequest) 
		ajaxRequest = new XMLHttpRequest();
	else
		ajaxRequest = new ActiveXObject("Microsoft.XMLHTTP");

	if (recfunct && isAsync) {
		ajaxRequest.onreadystatechange = function() {
			if (ajaxRequest.readyState==4 && ajaxRequest.status==200) {
				recfunct(ajaxRequest.responseText);
            }
		}
	}

	ajaxRequest.open("GET", filename, isAsync);
	ajaxRequest.send("random="+Math.random());
	
	if (!isAsync) {
		recfunct(ajaxRequest.responseText);
    }
}
