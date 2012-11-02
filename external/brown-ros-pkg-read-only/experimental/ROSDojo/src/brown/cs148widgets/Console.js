dojo.provide("cs148widgets.Console");

dojo.require("dijit._Widget");
dojo.require("cs148widgets.Utils");

dojo.declare("cs148widgets.Console", dijit._Widget, {
    
	postCreate: function() {
		dojo.addClass(this.domNode, "console");
		
		// Create the inner console div
		this.console = document.createElement('div');
		dojo.addClass(this.console, "consoletext");
		this.domNode.appendChild(this.console);
		
		// Create the button to clear the console
		this.clearButton = document.createElement('div');
		dojo.addClass(this.clearButton, "clearconsole");
		this.domNode.appendChild(this.clearButton);
		this.connect(this.clearButton, "onmouseup", "clear");
	},
	
	print: function(str) {
	    // First check to see the current scroll of the console
	    var scrollHeight = this.getScrollHeight();
	    
	    // Add the new text
		var text = document.createElement('div');
		text.appendChild(document.createTextNode(str));
		dojo.addClass(text, "consoleentry");
		this.console.appendChild(text);
		
		// Update the height then set the scroll appropriately
		this.refreshHeight();
		this.setScrollHeight(scrollHeight);
	},
	
	clear: function() {
	    console.log("click");
	    this.console.innerHTML = "";
	    this.refreshHeight();
	},
	
	setHeight: function(height) {
	    // First get the current scroll height
        var scrollHeight = this.getScrollHeight();
        
        // Modify the height
		dojo.style(this.domNode, "height", height+"px");
		this.refreshHeight();
        this.refreshWidth();
        
        // Only update the scroll if it was stuck to bottom
        if (scrollHeight==-1) {
            this.setScrollHeight(-1);
        }
	},
	
	refreshHeight: function() {	    
	    var maxHeight = dojo.contentBox(this.domNode).h;
        dojo.style(this.console, "height", "");
        var currentHeight = dojo.contentBox(this.console).h;
        var height = Math.min(maxHeight, currentHeight);
        dojo.style(this.console, "height", height+"px");
	},
    
    refreshWidth: function() {
        var width = dojo.contentBox(this.domNode).w;
        dojo.style(this.console, "width", width+"px");
    },
	
	getScrollHeight: function() {
        if (this.console.scrollHeight - this.console.scrollTop == this.console.clientHeight) {
            return -1;
        } else {
            return this.console.scrollTop;
        }
	},
	
	setScrollHeight: function(height) {
	    if (height==-1) {
            this.console.scrollTop = this.console.scrollHeight - this.console.clientHeight;	        
	    } else {
	        this.console.scrollTop = height;
	    }
	}
	
});