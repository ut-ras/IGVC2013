dojo.provide("mobile.TouchJoystick");

dojo.require("dijit._Widget");

/*
 * This widget is a simple 'joystick touch' widget.  It is intended to be a mixin for other widgets.  It registers
 * event handlers on the main domNode for touch events, and fires callbacks for various touch events.
 * The intention is to simplify touchy-feely widgets.
 */
dojo.declare("mobile.TouchJoystick", dijit._Widget , {

    // Internal variables.
	position: null,
	
	// Timing for click and doubleclick
	_last_mousedown: 0,
	_last_click: 0,
	_click_count: 0,

	postCreate : function() {
	    this.position = { x: 0, y: 0 };
        this.connect(this.domNode, "mousedown", "_joystick_down");
        this.connect(this.domNode, "touchstart", "_joystick_down");

        this.connect(this, "_joystick_down", "_tap_down");
        this.connect(this, "_joystick_finish", "_tap_up");
	},

	// Returns the touch position.  Position is centered based on initial touch position.
	// Values range from 0 to 1 (1 being full image width)
	getTouchLocation: function() {
	    return position;
	},

	/*
	 * Internals, don't touch :P
	 */

	_joystick_down: function(evt) {
		// Save the center of the joystick
        var center = { x: evt.clientX, y: evt.clientY };
        
        var move_callback = dojo.hitch(this, "_joystick_move", center);
        var end_callback = dojo.hitch(this, "_joystick_finish");
        
		// Connect up the other methods
        this._joystick_move_connection = this.connect(this.domNode, "mousemove", move_callback);
        this._joystick_touchmove_connection = this.connect(this.domNode, "touchmove", move_callback);
        
        this._joystick_up_connection = this.connect(this.domNode, "mouseup", end_callback);
        this._joystick_out_connection = this.connect(this.domNode, "mouseout", end_callback);
        this._joystick_touchend_connection = this.connect(this.domNode, "touchend", end_callback);
        this._joystick_touchcancel_connection = this.connect(this.domNode, "touchcancel", end_callback);
        
        // Disable browser's default behaviour
		evt.preventDefault();
		
		// Notify clients
		this.singleTouchStart();
	},

	_joystick_move: function(center, evt) {
	    // Update the current position
		this.position = {
			x: (evt.clientX - center.x) / this.domNode.clientWidth,
			y: (center.y - evt.clientY) / this.domNode.clientHeight
		};

        // Disable browser's default behaviour		
		evt.preventDefault();
	},

	_joystick_finish: function(evt) {
	    this.position = { x: 0, y: 0 };
	    
	    // Disconnect event handlers
        this.disconnect(this._joystick_move_connection);
        this.disconnect(this._joystick_up_connection);
        this.disconnect(this._joystick_out_connection);
        this.disconnect(this._joystick_touchmove_connection);
        this.disconnect(this._joystick_touchend_connection);
        this.disconnect(this._joystick_touchcancel_connection);

        // Disable browser's default behaviour
		evt.preventDefault();

        // Notify clients
		this.singleTouchStop();
	},
	
	_tap_down: function(evt) {    
	    var currentTime = new Date().getTime();
	    if (this._last_click + 500 < currentTime) {
	        this._click_count = 0;
	    }
	    this._last_mousedown = currentTime;
	},
	
	_tap_up: function(evt) {
        var currentTime = new Date().getTime();        
        if (this._last_mousedown + 500 > currentTime) {
            this._click_count += 1;
        }
        this._last_click = currentTime;
        if (this._click_count>=2) {
            this.doubleTap();
        }
	},


	/*
	 * CALLBACKS
	 */
	singleTouchStart: function() {},
	singleTouchStop: function() {},
	multiTouchStart: function() {},
	multiTouchStop: function() {},
    doubleTap: function() {},


	uninitialize: function() {

	},

});
