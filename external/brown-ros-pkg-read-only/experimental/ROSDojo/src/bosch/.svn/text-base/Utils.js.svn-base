dojo.provide("bosch.Utils");

//Load common CSS
var link = document.createElement('link');
link.rel = "stylesheet";
link.href = dojo.moduleUrl("bosch", "css/bosch.css");
document.body.appendChild(link);

// Load right click context menu css
var link2 = document.createElement('link');
link2.rel = "stylesheet";
link2.href = dojo.moduleUrl("bosch", "css/rightcontext.css");
document.body.appendChild(link2);


/*
link.href = dojo.moduleUrl("dojox", "grid/resources/Grid.css");
document.body.appendChild(link);
link.href = dojo.moduleUrl("dojox", "grid/resources/claroGrid.css");
document.body.appendChild(link);
*/
var loadVisualizationModule = function(filename) {
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl("bosch", filename), false);
	xhr.send('');
	var script = document.createElement('script');
	script.type = "text/javascript";
	script.text = xhr.responseText;
	document.getElementsByTagName('head')[0].appendChild(script);
}

loadVisualizationModule("ros/common.js");
loadVisualizationModule("ros/visualization/visualization.js");
loadVisualizationModule("ros/visualization_widgets/colorpicker.js");
loadVisualizationModule("spidergl/spidergl.js");
loadVisualizationModule("ros/binaryparser/binaryparser.js");
loadVisualizationModule("ros/b64codec/b64codec.js");

dojo.declare("bosch.Utils", null, {});

// Returns an object with { left, right, top, bottom, width, height } indicating how much of the
// background can be cropped
bosch.Utils.getBounds = function(px, width, height, r, g, b) {	
	var top = bosch.Utils.top(px, width, height, r, g, b);
	var bottom = bosch.Utils.bottom(px, width, height, r, g, b);
	var left = bosch.Utils.left(px, width, height, r, g, b);
	var right = bosch.Utils.right(px, width, height, r, g, b);
	
	return { top: top, bottom: bottom, left: left, right: right, width: (right-left), height: (bottom-top) };
}

// Determines the blank space at the top of the image and returns the y value
// of the first line of interesting image
bosch.Utils.top = function(px, width, height, r, g, b) {	
	var i = 0;
	while (i < height && bosch.Utils.isRowBlank(px, width, r, g, b, i)) {
		i++;
	}
	if (i==height) {
		i = 0;
	}
	return i;
}


//Determines the blank space at the bottom of the image and returns the y value
//of the final line of interesting image
bosch.Utils.bottom = function(px, width, height, r, g, b) {
	var i = height-1;
	while (i > 0 && bosch.Utils.isRowBlank(px, width, r, g, b, i)) {
		i--;
	}
	return i;
}

//Determines the blank space at the left of the image and returns the x value
//of the first line of interesting image
bosch.Utils.left = function(px, width, height, r, g, b) {
	var i = 0;
	while (i < width && bosch.Utils.isColBlank(px, width, r, g, b, i)) {
		i++;
	}
	if (i==width) {
		i = 0;
	}
	return i;
}

//Determines the blank space at the right of the image and returns the x value
//of the final line of interesting image
bosch.Utils.right = function(px, width, height, r, g, b) {
	var i = width-1;
	while (i > 0 && bosch.Utils.isColBlank(px, width, r, g, b, i)) {
		i--;
	}
	return i;
}

bosch.Utils.isRowBlank = function(px, w, r, g, b, lineNumber) {
	var i = lineNumber*w*4;
	var max = (lineNumber+1)*w*4;
	for (; i < max; i+=4) {
		if (!bosch.Utils.pixelMatches(px, r, g, b, i)) {
			return false; 
		}
	}
	return true;
}

bosch.Utils.isColBlank = function(px, w, r, g, b, colNumber) {
	var i = colNumber*4;
	var max = px.length;
	for (; i < max; i+=w*4) {
		if (!bosch.Utils.pixelMatches(px, r, g, b, i)) {
			return false; 
		}
	}
	return true;	
}

bosch.Utils.pixelMatches = function(px, r, g, b, i) {
	if (px[i]==r && px[i+1]==g && px[i+2]==b) {
		return true; 
	}
	return false;
}
