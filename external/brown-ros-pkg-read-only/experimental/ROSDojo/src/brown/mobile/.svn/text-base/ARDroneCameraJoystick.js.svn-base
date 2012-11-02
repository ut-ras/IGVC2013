dojo.provide("mobile.ARDroneCameraJoystick");

dojo.require("mobile.TouchJoystick");
dojo.require("roswidgets.MJPEGViewer");
dojo.require("rosdojo.rosdojo");

dojo.require("dijit._Widget");

/*
 * This widget is a simple 'joystick touch' widget.  It is intended to be a mixin for other widgets.  It registers
 * event handlers on the main domNode for touch events, and fires callbacks for various touch events.
 * The intention is to simplify touchy-feely widgets.
 */
dojo.declare("mobile.ARDroneCameraJoystick", [roswidgets.MJPEGViewer, mobile.TouchJoystick] , {

	topic: "/ardrone/image_raw",
	rate: 100,
	timer: null,
	airborne: false,
	
	postCreate: function() {
		this.inherited(arguments);

        this.connect(this, "singleTouchStart", "touchStart");
        this.connect(this, "singleTouchStop", "touchStop");
	},
	
	startup: function() {
		this.createImg("192.168.160.103");
	},
	
	touchStart: function() {	    
	    // Start publishing
	    this.publish();
	},
	
	touchStop: function() {	    	    
	    // Publish the reset
	    this.publish();
	    
	    // Stop publishing
        this.stopPublishing();
	},
	
	doubleTap: function() {
	    if (this.airborne) {
	        this.land();
	    } else {
	        this.takeOff();
	    }
	    this.airborne = !this.airborne;
	},

    publish: function() {
        var message = {
            linear: { x: this.position.y*0.25, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: -this.position.x }
        };
        ros.publish('/cmd_vel', 'geometry_msgs/Twist', dojo.toJson(message));
        
        // Queue up another timer
        window.clearTimeout(this.timer);
        this.timer = window.setTimeout(dojo.hitch(this, "publish"), this.rate);
    },
    
    takeOff: function() { ros.publish("/ardrone/takeoff",'std_msgs/Empty', '{}'); },
    land: function() { ros.publish("/ardrone/land",'std_msgs/Empty', '{}'); },
    
    stopPublishing: function() {
        window.clearTimeout(this.timer);        
    }

});
