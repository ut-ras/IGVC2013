function log(msg) {
    $("#console").html($("#console").html() + msg + "\r\n");
}

(function($){
    $(document).ready(function(){
	///////////////
	// Parse querystring
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
    	    function move(x,z) {
    		ros.publish('/cmd_vel',
			    'geometry_msgs/Twist',
			    '{"linear":{"x":' + x + ',"y":0,"z":0}, "angular":{"x":0,"y":0,"z":' + z + '}}');
    	    }
	    var x = 0;
	    var z = 0;
	    var prevX = 0;
	    var prevZ = 0;
	    setInterval(function() {
		if (x != prevX || z != prevZ) {
		    move(x,z);
		    prevX = x;
		    prevZ = z;
		}
	    }, 250) // ms interval -> 4 hz
	    ///////////////
	    // Control by "on-screen trackpad".  Leaving pad halts robot.
	    // ?todo: limit rate? (hz)
	    $("#robotController").mousemove(function(e){
		// center is at 0,0
		// TODO: .offset only works on Chrome4, not Firefox4
		var xFromCenter = e.offsetX - 200;
		var yFromCenter = 150 - e.offsetY; // invert y so up is +
		var fromCenterCoords = "( " + xFromCenter + ", " + yFromCenter + " )";
		var invSensitivity = $("#invSensitivity").val()
		x = (yFromCenter/300) * (100/invSensitivity); // m/sn
		z = (-xFromCenter/100) * (100/invSensitivity); // right turns CW; rad/s
		// For debugging:
		// $("#positionCoords").text("Position: " + fromCenterCoords + " " + 100/invSensitivity);
	    });
	    $("#robotController").mouseleave(function(e){
		x = 0;
		z = 0;
	    });
	});
    });
})(jQuery);
