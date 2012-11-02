dojo.provide("controller.ARDroneTeleop");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");
dojo.require("dijit.form.Button");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");
dojo.require("controller.Utils");

dojo.declare("controller.ARDroneTeleop", [ dijit._Widget, dijit._Templated ], {

	templateString : dojo.cache("controller","templates/ARDroneTeleop.html"),
	
	// Internal variables
	linear: null,
	angular: null,
	buttons: null,

	postCreate : function() {
	    // Set the default linear and angular variables
	    this.reset();
	    
	    // Create the controls
		this.createButtons();

		// Connect keyboard controls
		this.connect(window, 'keydown', "onKeyDown");
		this.connect(window, 'keyup', "onKeyUp");
		
		// Connect up events to enable/disable controls when the connection changes
        this.connect(ros, "onOpen", "onOpen");
        this.connect(ros, "onClose", "onClose");
        
        // If connection already up, manually call open
        if (ros.available()) {
        	this.onOpen();
        }
	},

	createButtons : function() {
		this.buttons = [];
		
		// Takeoff and Land are special cases. They only need to fire a single event
		var takeOffButton = new dijit.form.Button({ label: "Take Off", disabled: true },this.takeOffAttach);
		var landButton = new dijit.form.Button({ label: "Land", disabled: true },this.landAttach);
		this.connect(takeOffButton, "onClick", "takeOff");
		this.connect(landButton, "onClick", "land");
		this.buttons.push(takeOffButton);
		this.buttons.push(landButton);

		// Create buttons for the rest of the controls
		this.buttons.push(this.createButton(this.forwardAttach, "Forward", 38));
		this.buttons.push(this.createButton(this.reverseAttach, "Reverse", 40));
		this.buttons.push(this.createButton(this.turnleftAttach, "Turn Left", 37));
		this.buttons.push(this.createButton(this.turnrightAttach, "Turn Right", 39));
		this.buttons.push(this.createButton(this.climbAttach, "Climb", 33));
		this.buttons.push(this.createButton(this.descendAttach, "Descend", 34));
		this.buttons.push(this.createButton(this.strafeleftAttach, "Strafe Left", 90));
		this.buttons.push(this.createButton(this.straferightAttach, "Strafe Right", 88));
	},
	
	createButton: function(attachPoint, label, keyCode) {
		var button = new dijit.form.Button({ label: label, disabled: true }, attachPoint);
		var pressed = dojo.hitch(this, "onMouseDown", keyCode);
		var unpressed = dojo.hitch(this, "onMouseUp", keyCode);
		this.connect(button, "onMouseDown", pressed);
		this.connect(button, "onMouseUp", unpressed);
		this.connect(button.domNode, "touchstart", pressed); /* for ipad */
		this.connect(button.domNode, "touchend", unpressed); /* for ipad */
		return button;
	},
	
	onOpen: function() {
		for (var i = 0; i < this.buttons.length; i++) {
			this.buttons[i].setDisabled(false);
		}
	},
	
	onClose: function() {
		for (var i = 0; i < this.buttons.length; i++) {
			this.buttons[i].setDisabled(true);
		}
	},
	
	takeOff: function() { ros.publish("/ardrone/takeoff",'std_msgs/Empty', '{}'); },
	land: function() { ros.publish("/ardrone/land",'std_msgs/Empty', '{}'); },

	onKeyDown : function(event) { this.movePressed(event.keyCode); },
	onKeyUp : function(event) { this.moveUnPressed(event.keyCode); },
    onMouseDown: function(keycode, event) { this.movePressed(keycode); event.preventDefault(); },
    onMouseUp: function(keycode, event) { this.moveUnPressed(keycode); event.preventDefault(); },
	
	movePressed: function(keyCode, evt) {
		this.handleKey(keyCode);
		this.pub();
	},
	
	moveUnPressed: function(keyCode, evt) {
		this.reset();
		this.pub();
	},

	handleKey: function(keyCode) {
		switch (keyCode) {
			case 37: /* left */ this.angular.z = 1; break;
			case 39: /* right */ this.angular.z = -1; break;
			case 38: /* up */ this.linear.x = .25; break;
			case 40: /* down */ this.linear.x = -.25; break;
			case 90: /* z */ this.linear.y = .25; break;
			case 88: /* x */ this.linear.y = -.25; break;
			case 33: /* pgup */ this.linear.z = 0.15; break;
			case 34: /* pgdn */ this.linear.z = -0.15; break;
			case 36: /* home */ this.takeOff(); break;
			case 35: /* end */ this.land(); break;
		}
	},
	
	reset: function() {
        this.linear = { x: 0, y: 0, z: 0 };
        this.angular = { x: 0, y: 0, z: 0 };
	},

	pub: function() {
		ros.publish('/cmd_vel', 'geometry_msgs/Twist', dojo.toJson({ linear: this.linear, angular: this.angular }));
	}
});
