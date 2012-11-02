dojo.provide("controller.TwistTeleop");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");
dojo.require("dijit.form.Button");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("controller.TwistTeleop", [ dijit._Widget, dijit._Templated], {

	templateString : dojo.cache("controller","templates/TwistTeleop.html"),

	topic: '/cmd_vel',
	type : '/geometry_msgs/Twist',
	x:0,
	y:0,
	z:0,
	theta:0,

	postCreate : function() {
		this.connect(window,'keydown',"keydown");
		this.connect(window,'keyup',"keyup");
	},

	keydown : function(event)
	{
		this.handleKey(event.keyCode,true);
	},

	keyup : function(event)
	{
		this.handleKey(event.keyCode,false);
	},

	pub : function() {
		var linear = { x: this.x, y: this.y, z: this.z };
		var angular = { x: 0, y: 0, z: this.theta };
		ros.publish('/cmd_vel', 'geometry_msgs/Twist', dojo.toJson({ linear: linear, angular: angular }));
	},

	handleKey : function(code, down) {
		var scale = 0;
		if (down == true) {
			scale = 1;
		}
		switch (code) {
		case 37:
			//left
			this.theta = 1 * scale;
			break;
		case 38:
			//up
			this.x = .25 * scale;
			break;
		case 39:
			//right 
			this.theta = -1 * scale;
			break;
		case 40:
			//down
			this.x = -.25 * scale;
			break;
		case 90:
			//z
			this.y = .25 * scale;
			break;
		case 88:
			//x
			this.y = -.25 * scale;
			break;
		}
		this.pub();
	},
});
