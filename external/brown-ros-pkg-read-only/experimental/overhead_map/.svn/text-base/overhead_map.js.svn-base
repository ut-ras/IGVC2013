///////////////////////////////////////////////////////////////////////////////
// Game viewer
///////////////
// Brian Thomas (brianjaythomas@gmail.com), 2010/09/03-20
// Depends on: rosjs, jQuery, jquery.query plugin, and om_msgs
// Connects to ROS and listens for topic "/overhead_map_objs" (of type
//   Overhead_Map_Msgs), which provides a list of objects including points,
//   lines, arrows, and images, and an associated tuple with each one of them:
//     * point ("point", [x y])         draws a point at (x, y)
//     * line  ("line", [x0 y0 x1 y1])  draws a line from (x0, y0) to (x1, y1)
//     * arrow ("arrow", [x0 y0 x1 y1]) draws an arrow (x0, y0) --> (x1, y1)
//     * image ("imname", [x y th])     draws in image centered at (x,y),
//                                        rotated theta radians.  location of
//                                        image is 'icons/imname.png'.
//   All coordinates are in the "Ideal" coordinate system.
//
// "Ideal" Coordinates:
//        ^+y
//        |
//        |
// (0,0) -|----> +x
// Theta == 0 faces --> ; + moves counterclockwise.  (Theta is in radians.)
//
// The on-screen drawing draws a border the size of half the largest image's
//   diagonal, so that partially-out-of-bounds robots will still display
//   completly onscreen.  (Note that the ideal coordinates (0,0) are on the
//   "out-of-bounds border", not the html canvas's edge.)
///////////////////////////////////////////////////////////////////////////////

///////////////
// Wait for document loaded, and do ROS stuff
(function($){
    $(document).ready(function(){
	///////////////
	// Parse querystring
	var mapToUse = $.query.get("map");
	mapToUse = (mapToUse == "") ? "soccer" : mapToUse; // default on "soccer"
	var address = $.query.get("address"); // address to connect to rosjs
	address = (address == "") ? "127.0.0.1" : address; // default on local hosting
	///////////////
	// Connect to ROS and Topics
	var ros = new ROS("ws://"+address+":9090");
	log("Attempting to connect...");
	ros.setOnClose(function(e) {
    	    log('Connection closed.');
	});
	ros.setOnError(function(e) {
    	    log('Error!');
	});
	ros.setOnOpen(function(e) {
	    log('Connection open.');
	    go($,ros,mapToUse);
	});
    });
})(jQuery);

// $ is the jQuery object
// ros is the connected instance of ROS
// mapToUse is a string
function go($,ros,mapToUse) {
    // Things we want to draw are received from the overhead_map_objs topic.
    var overhead_map_objs;
    ros.addHandler("/overhead_map_objs",function(om_objs) {
	overhead_map_objs = om_objs.objs;
    });
    ros.callService('/rosjs/subscribe','["/overhead_map_objs",0]',
		    function(rsp) {});
    ///////////////
    // Load images and settings from server
    var centerLogo = 'logos/center.png';
    var bottomRightLogo = 'logos/bottom-right.png';
    // vv Firefox 4 currently doesn't have svg support (Firefox bug)
    var background = 'backgrounds/'+mapToUse+'.png';
    // vv This currently doesn't work in Chrome under file://* (Chrome bug)
    $.get("backgrounds/"+mapToUse+".xml", function(xml) {
	var MAX_GIVEN_WIDTH = parseFloat($(xml).find("width").text());
	var MAX_GIVEN_HEIGHT = parseFloat($(xml).find("height").text());
	var BORDER_COLOR = $(xml).find("border_color").text();
	var MARKER_COLOR = $(xml).find("marker_color").text();
	$.get("icons/icons.xml", function(xml) {
	    var imageList = [background, centerLogo, bottomRightLogo, 'icons/default.png'];
	    $(xml).find("icon").each(function() {
		imageList.push('icons/'+$(this).text());
	    });
	    loadImages(imageList, function(images) {
		///////////////
		// Drawing loop
		var board = $('#overhead_map_board')[0].getContext("2d");
		setInterval(function() {
		    // Need to able to draw robot fully on field at
		    //   (MAX_GIVEN_WIDTH, MAX_GIVEN_HEIGHT).
		    // Canvas cannot be resized, so canvas width/height
		    //   is an absolute max.
		    // 20 is a matting fudge factor

		    var MAXWIDTH = Math.min($("#overhead_map_board").width(),
					    $(window).width() - 20);
		    var MAXHEIGHT = Math.min($("#overhead_map_board").height(),
					     $(window).height() - 20);
		    // TODO actually find max for "iconic" images
		    var MAXIMAGEDIAG = Math.sqrt(Math.pow(images['icons/default.png'].width/2,2),
						 Math.pow(images['icons/default.png'].height/2,2));
		    // Solution to equations:
		    //  Border = MaxImageDiag * Scale
		    //  Scale = (WindowSize - 2*Border) / maxWindowSize
		    //  is: Border = WindowSize / (maxWindowSize/MaxImageDiag + 2)
		    var BORDER = Math.min(MAXWIDTH / ($("#overhead_map_board").width()/MAXIMAGEDIAG + 2),
					  MAXHEIGHT / ($("#overhead_map_board").height()/MAXIMAGEDIAG + 2));
		    var PIXELS_PER_UNIT = Math.min((MAXWIDTH - 2*BORDER) / MAX_GIVEN_WIDTH,
						   (MAXHEIGHT - 2*BORDER) / MAX_GIVEN_HEIGHT);
		    var BOARD_WIDTH = MAX_GIVEN_WIDTH * PIXELS_PER_UNIT + 2*BORDER;
		    var BOARD_HEIGHT = MAX_GIVEN_HEIGHT * PIXELS_PER_UNIT + 2*BORDER;
		    var INNER_BOARD_WIDTH = BOARD_WIDTH - 2*BORDER;
		    var INNER_BOARD_HEIGHT = BOARD_HEIGHT - 2*BORDER;
		    var SCALE = Math.max(INNER_BOARD_WIDTH/$("#overhead_map_board").width(),
					 INNER_BOARD_HEIGHT/$("#overhead_map_board").height());
		    // Drawing constants
		    var lineWidth = 5 * SCALE; // pixels
		    var strokeStyle = MARKER_COLOR;
		    var fillStyle = MARKER_COLOR;

		    function idealXtoCanvasX(x) {
			// x coordinate frame is the same except for scaling
			return BORDER + x*PIXELS_PER_UNIT;
		    }
		    function idealYtoCanvasY(y) {
			// y coordinate frame is different.  In
			//   graphics coordinates, + is down, with 0,0
			//   at top left border. In ideal coordinates,
			//   + is up, and 0,0 is at bottom left border.
			return (BOARD_HEIGHT - BORDER) - y*PIXELS_PER_UNIT;
		    }
		    function idealThetaToCanvasTheta(theta) {
			// Graphics theta rotates CW; ideal rotates CCW
			//   (Both are in radians.)
			return -theta;
		    }
		    function drawPointIdeal(x,y) {
			// x, y in "ideal coordinates"
			// draws point at (x,y)
			var pointRadius = 5 * SCALE; // pixels
			board.lineWidth = lineWidth;
			board.strokeStyle = strokeStyle;
			board.fillStyle = fillStyle;
			board.beginPath();
			board.arc(idealXtoCanvasX(x),
				  idealYtoCanvasY(y),
				  pointRadius,
				  0,
				  Math.PI*2,
				  true);
			board.closePath();
			board.stroke();
			board.fill();
		    }
		    function drawLineIdeal(x0,y0,x1,y1) {
			// x, y in "ideal coordinates"
			// draws line from (x0,y0) to (x1,y1)
			board.lineWidth = lineWidth;
			board.strokeStyle = strokeStyle;
			board.fillStyle = fillStyle;
			board.beginPath();
			board.moveTo(idealXtoCanvasX(x0),
				     idealYtoCanvasY(y0));
			board.lineTo(idealXtoCanvasX(x1),
				     idealYtoCanvasY(y1));
			board.closePath();
			board.stroke();
		    }
		    function drawArrowIdeal(x0,y0,x1,y1) {
			// x, y in "ideal coordinates"
			// draws arrow (x0,y0) --> (x1,y1)
			drawLineIdeal(x0,y0,x1,y1);
			// Draw the two arrows 45 degrees from the line
			var arrowLineLength = 20 * SCALE; // pixels
			var centerAngle = Math.atan2(y1-y0,x1-x0);
			var leftAngle = centerAngle - Math.PI/4;
			var rightAngle = centerAngle + Math.PI/4;
			board.lineWidth = lineWidth;
			board.strokeStyle = strokeStyle;
			board.fillStyle = fillStyle;
			board.beginPath();
			// Note: +/- inverted for sin because the y
			//   coordinate system is inverse in canvas
			//   rendering, but all Math functions are
			//   for "ideal" system.
			// left line (extra movement makes it look pointy at end)
			board.moveTo(
			    idealXtoCanvasX(x1) + lineWidth/2 * Math.cos(leftAngle),
			    idealYtoCanvasY(y1) - lineWidth/2 * Math.sin(leftAngle));
			board.lineTo(
			    idealXtoCanvasX(x1) - arrowLineLength * Math.cos(leftAngle),
			    idealYtoCanvasY(y1) + arrowLineLength * Math.sin(leftAngle));
			// right line
			board.moveTo(
			    idealXtoCanvasX(x1) + lineWidth/2 * Math.cos(rightAngle),
			    idealYtoCanvasY(y1) - lineWidth/2 * Math.sin(rightAngle));
			board.lineTo(
			    idealXtoCanvasX(x1) - arrowLineLength * Math.cos(rightAngle),
			    idealYtoCanvasY(y1) + arrowLineLength * Math.sin(rightAngle));
			// right line
			board.closePath();
			board.stroke();
		    }
		    function drawImageIdeal(image,x,y,theta) {
			// x, y, theta in "ideal coordinates"
			// draws image, scaled proportional to board
			//   shrinkage, and centered
			var imwidth = image.width * SCALE;
			var imheight = image.height * SCALE;
			board.save();
			board.translate(idealXtoCanvasX(x), idealYtoCanvasY(y));
			board.rotate(idealThetaToCanvasTheta(theta));
			// Draw image
			board.drawImage(image, -imwidth/2, -imheight/2, imwidth, imheight);
			board.restore();
		    }

		    // clear space
		    board.fillStyle = "white";
		    board.fillRect(0, 0, $("#overhead_map_board").width(), $("#overhead_map_board").height()); // size of canvas in HTML
		    // outside border
		    board.fillStyle = BORDER_COLOR;
		    board.fillRect(0, 0, BOARD_WIDTH, BOARD_HEIGHT);
		    // background image
		    board.drawImage(images[background], BORDER, BORDER, INNER_BOARD_WIDTH, INNER_BOARD_HEIGHT);

		    // logos
		    // brown -- center
		    drawImageIdeal(images[centerLogo], MAX_GIVEN_WIDTH/2, MAX_GIVEN_HEIGHT/2, 0);
		    // rlab -- just above the bottom-right corner
		    var bottomRightLogoWidth = images[bottomRightLogo].width * SCALE;
		    var bottomRightLogoHeight = images[bottomRightLogo].height * SCALE;
		    board.drawImage(images[bottomRightLogo],
				    BOARD_WIDTH - BORDER - bottomRightLogoWidth - 5*SCALE,
				    BOARD_HEIGHT - BORDER - bottomRightLogoHeight - 5*SCALE,
				    bottomRightLogoWidth,
				    bottomRightLogoHeight);

		    // points, lines, arrows, icons
		    for (i=0; i<overhead_map_objs.length; i++) {
			if (overhead_map_objs[i].name == "point") {
			    drawPointIdeal(overhead_map_objs[i].tuple[0], // x
				    	   overhead_map_objs[i].tuple[1]); // y
			}
			else if (overhead_map_objs[i].name == "line") {
			    drawLineIdeal(overhead_map_objs[i].tuple[0], // x0
				    	  overhead_map_objs[i].tuple[1], // y0
				    	  overhead_map_objs[i].tuple[2], // x1
				    	  overhead_map_objs[i].tuple[3]); // y1
			}
			else if (overhead_map_objs[i].name == "arrow") {
			    drawArrowIdeal(overhead_map_objs[i].tuple[0], // x0
				    	   overhead_map_objs[i].tuple[1], // y0
				    	   overhead_map_objs[i].tuple[2], // x1
				    	   overhead_map_objs[i].tuple[3]); // y1
			}
			else {
			    var filename='icons/'+overhead_map_objs[i].name+'.png'
			    var image = images[filename] ?
				images[filename] :
				images['icons/default.png'];
			    drawImageIdeal(image,
				    	   overhead_map_objs[i].tuple[0], // x
				    	   overhead_map_objs[i].tuple[1], // y
				    	   overhead_map_objs[i].tuple[2]); // theta
			}
		    }

		},
			    250); // ms interval (--> ~4fps)
		// /Drawing loop
		///////////////
	    });
	});
    });
}

///////////////
// Helper functions
function log(msg) {
    $("#overhead_map_console").text(msg + "\r\n" + $("#overhead_map_console").text());
}

function loadImages(filepaths, onload_function) {
    // filepaths is an array of Strings representing filepaths.
    // onload_function is a function taking one argument, a (new) array of
    //   "Image"s, which is to be executed after all "Image"s are loaded.
    //   This array is overloaded so that an image can both be referred to
    //   by number (for looping) or by filepath.
    var images = new Array(filepaths.length);
    if (filepaths.length == 0) {
	onload_function(images);
    }
    else {
	// Load each image in series, then call onload_function when all
	//   images are loaded.
	function chainLoad(i) {
	    images[i] = new Image();
	    images[filepaths[i]] = images[i];
	    images[i].src = filepaths[i];
	    if(i == filepaths.length - 1) {
	    	onload_function(images);
	    }
	    else {
	    	images[i].onload = function() {
	    	    // setTimeout used to prevent tail-end recursion, which
		    //   will cause nesting because Javascript doesn't
		    //   require tail-call optimization.
		    setTimeout(function(){chainLoad(i+1);},0);
	    	}
	    }
	}
	chainLoad(0);
    }
}


