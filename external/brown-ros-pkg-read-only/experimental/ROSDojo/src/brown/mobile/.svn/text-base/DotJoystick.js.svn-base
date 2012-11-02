dojo.provide("mobile.DotJoystick");

dojo.require("dijit._Widget");
dojo.require("dojox.gfx");
dojo.require("dojox.gfx.move");

/*
 * This widget is a simple 'joystick touch' widget.  It is intended to be a mixin for other widgets.  It registers
 * event handlers on the main domNode for touch events, and fires callbacks for various touch events.
 * The intention is to simplify touchy-feely widgets.
 */
dojo.declare("mobile.DotJoystick", dijit._Widget , {

	center: null,
	position: { x: 0, y: 0 },

	postCreate : function() {		
        this.connect(this.domNode, "mousedown", "_joystick_down");
        this.connect(this.domNode, "touchstart", "_joystick_down");
	},

	// Returns true if the user is currently touching the widget
	isTouching: function() {
		
	},

	// Returns an average of all the currently touched positions.
	// Rather than screen coords, the returned location is CARTESIAN
	// with respect to the ORIGINAL down location
	getTouchLocation: function() {

	},

  createSurface : function()
  {
    var div = document.createElement('div');
    div.style.position = "absolute";
    div.style.top = "0px";
    div.style.left = "0px";
    var surface = dojox.gfx.createSurface(div,this.width,this.height);

    this.circle_x = this.width/2;
    this.circle_y = this.height/2;
    this.circle_r = 22;

    this.circle = surface.createCircle({cx:this.circle_x, cy:this.circle_y, r:this.circle_r})
                  .setFill([0,0,255,0.5])
                  .setStroke({color:'blue',width:2})
                  ;
    this.circle_move = new dojox.gfx.Moveable(this.circle);
    this.surface = surface;

    this.domNode.appendChild(div);
  },

  resizeSurface : function(width,height)
  {
    this.surface.setDimensions(width,height);
    this.surface.remove(this.circle);
    this.circle = this.surface.createCircle({cx:width/2, cy:height/2, r:this.circle_r})
                  .setFill([0,0,255,0.5])
                  .setStroke({color:'blue',width:2})
                  ;
    this.circle_x= width/2;
    this.circle_y= height/2;
  },

	/*
	 * Internals, don't touch :P
	 */

	_joystick_down: function(evt) {

    if(Math.abs(evt.clientX - this.circle_x) < this.circle_r &&
        Math.abs(evt.clientY - this.circle_y < this.circle_r))
    {
      console.log("here");
    }

		this.center = { x: evt.clientX, y: evt.clientY };
        this._joystick_move_connection = this.connect(this.domNode, "mousemove", "_joystick_move");
        this._joystick_up_connection = this.connect(this.domNode, "mouseup", "_joystick_finish");
        this._joystick_out_connection = this.connect(this.domNode, "mouseout", "_joystick_finish");
        this._joystick_touchmove_connection = this.connect(this.domNode, "touchmove", "_joystick_move");
        this._joystick_touchend_connection = this.connect(this.domNode, "touchend", "_joystick_finish");
        this._joystick_touchcancel_connection = this.connect(this.domNode, "touchcancel", "_joystick_finish");
		evt.preventDefault();
		this.singleTouchStart(this.center);
	},

	_joystick_move: function(evt) {
		this.position = {
			x: evt.clientX - this.center.x,
			y: this.center.y - evt.clientY
		};
		console.log(this.position);
		evt.preventDefault();
	},

	_joystick_finish: function(evt) {
        this.disconnect(this._joystick_move_connection);
        this.disconnect(this._joystick_up_connection);
        this.disconnect(this._joystick_out_connection);
        this.disconnect(this._joystick_touchmove_connection);
        this.disconnect(this._joystick_touchend_connection);
        this.disconnect(this._joystick_touchcancel_connection);
		evt.preventDefault();
		this.singleTouchStop();
	},


	/*
	 * CALLBACKS
	 */
	singleTouchStart: function(center) {},
	singleTouchStop: function() {},
	multiTouchStart: function() {},
	multiTouchStop: function() {},


	uninitialize: function() {

	},

});
