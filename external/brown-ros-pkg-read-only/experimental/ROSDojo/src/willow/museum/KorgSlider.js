dojo.provide("museum.KorgSlider");

dojo.require("dijit._Widget");

dojo.declare("museum.KorgSlider", dijit._Widget, {
	
	// Parameters
	value: 0,
	
	// Internal variables
	_dragging: false,
	_clickoffset: 0,
	_sliderheight: 0,
	
	postCreate: function() {
		dojo.addClass(this.domNode, "korg-slider");		
		
		// Add a decorative center line
		var line = document.createElement('div');
		dojo.addClass(line, "korg-slider-line");
		this.domNode.appendChild(line);
		
		// Create the knob as a div
		this.knob = document.createElement('div');
		dojo.addClass(this.knob, "korg-slider-knob");
		this.domNode.appendChild(this.knob);
		
		// Connect the mouse events to move the knob
		this.connect(this.knob, "onmousedown", "_mouseDown");
		this.connect(window, "onmousemove", "_mouseMove");
		this.connect(window, "onmouseup", "_mouseUp");
		
		// Set the default value of 0
		window.setTimeout(dojo.hitch(this, "_setValue", this.value), 0);
	},
	
	// Value must be between -1 and 1
	setValue: function(value) {
		if (this.value!=value && !this._dragging) {
			this._setValue(value);
		}
	},
	
	_mouseDown: function(evt) {
		this._clickoffset = evt.clientY - dojo.position(this.knob).y + dojo.position(this.domNode).y;
		this._dragging = true;
		this._sliderheight = dojo.position(this.domNode).h - dojo.position(this.knob).h;
		evt.preventDefault();
	},
	
	_mouseMove: function(evt) {
		if (this._dragging) {
			var value = evt.clientY - this._clickoffset; // get the absolute pixel value
			value = 1 - (2 * value / this._sliderheight); // scale to -1 to 1
			if (value < -1) {
				value = -1;
			}
			if (value > 1) {
				value = 1;
			}
			this._setValue(value);
			evt.preventDefault();
			this.onSliderMoved();
		}
	},
	
	_mouseUp: function(evt) {
		this._dragging = false;
		this._sliderheight = null;
	},
	
	_setValue: function(value) {
		this.value = value;
		value = 1 - (value + 1) / 2; // Set to between 0 and 1
		if (!this._sliderheight) {
			// Set the knob starting location
			this._sliderheight = dojo.position(this.domNode).h - dojo.position(this.knob).h;
		}
		var top = this._sliderheight*value;
		dojo.style(this.knob, "top", top+"px");
	},
	
	// Connect to this event to be notified when the value of this slider changes
	onSliderMoved: function(value) {}
	
});