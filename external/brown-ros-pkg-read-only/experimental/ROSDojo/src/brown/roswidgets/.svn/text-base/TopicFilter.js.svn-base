dojo.provide("roswidgets.TopicFilter");

dojo.require("dijit.form.TextBox");
dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.TopicFilter", [ dijit._Widget ], {
    
	// Parameters
    /* none */
        
    // Internal variables
    filterstring: "",
    filterChangedTimeout: null,
    filterChangedWait: 300,
    
    postCreate : function() {    	
        // Create and attach the textbox
        var tbdiv = document.createElement('div');
        this.domNode.appendChild(tbdiv);
        this.textbox = new dijit.form.TextBox({ "spellcheck": false, "autocomplete": "off", "selectOnClick": true }, tbdiv);
        dojo.addClass(this.domNode, "topicfilter");
        
        // Add the question mark
        var img = document.createElement('img');
        img.src = dojo.moduleUrl("roswidgets", "img/questionmark.png");
        
        this.helpAnchor = document.createElement('a');
        dojo.addClass(this.helpAnchor, "helpicon");
        this.helpAnchor.href = "javascript:void(0)";
        this.helpAnchor.appendChild(img);
        this.connect(this.helpAnchor, "onclick", "showHelp");
        this.domNode.appendChild(this.helpAnchor);
        
        // Set the default value
        this._setDefault();
        
        // Connect up some functions
        this.connect(this.textbox, "onKeyUp", "_handleFilterChanged");
        this.connect(this.textbox, "onBlur", "_onBlur");
    },
    
    _setDefault: function() {
        this.textbox.setValue("Enter a filter string here");  
		dojo.addClass(this.domNode, "nofilter");  	
    },
    
    _handleFilterChanged: function(evt) {
        var newfilterstring = this.textbox.getValue();
        
        if (this.filterstring!=newfilterstring) {
            // Clear the timeout for sending the filter changed event
            window.clearTimeout(this.filterChangedTimeout);
            
            // Save the filter
        	this.filterstring = this.textbox.getValue();
        	
        	// Note that we now have a filter
    		dojo.removeClass(this.domNode, "nofilter");
    		
    		// Queue the filter changed call
    		this.filterChangedTimeout = window.setTimeout(dojo.hitch(this, "onFilterChanged", this.filterstring), this.filterChangedWait);
        }
    },
    
    _onBlur: function() {
    	if (!this.filterstring || this.filterstring=="") {
    		this._setDefault();
    	}
    },
    
    showHelp: function() {
        if (!this.showingTooltip) {
            // Just show the tooltip
            var message = "<div class='topicfilterhelp'>Enter a filter string here to filter the fields displayed on incoming messages.<br><br>";
            message += "- Type <i>field</i> to display a specific field and then <i>field.field</i> to access sub-fields.<br>";
            message += "- Array elements and ranges can be accessed using rudimentary Python-like notation: <i>array[i]</i>, <i>array[i:j]</i>, <i>array[i:]</i><br>";
            message += "- Multiple filter strings can be separated by spaces, and are treated as a union.<br><br>";
            message += "Examples:<br>";
            message += "<span class='key'>Topic</span>: <span class='topic'>/rosout</span><span class='value'>Filter</span>: <span class='example'>topics[0:3]</span><br>";
            message += "<span class='key'>Topic</span>: <span class='topic'>/tf</span><span class='value'>Filter</span>: <span class='example'>transforms[0].transform.translation</span><br>";
            message += "<span class='key'>Topic</span>: <span class='topic'>/diagnostics</span><span class='value'>Filter</span>: <span class='example'>header.stamp status[0].message</span><br></div>";
            
            this.showingTooltip = true;
            dijit.showTooltip(message, this.helpAnchor, [ "after", "above", "below", "before" ], false, "");
        } else {
            // The tooltip may be open, or it may have closed of its own accord.
            this.showingTooltip = false;
            if (dijit._masterTT.aroundNode==this.helpAnchor) {
                // Tooltip is still open, so hide it
                dijit.hideTooltip(this.helpAnchor);
            } else {
                // Tooltip had actually closed, so reopen it.
                this.showHelp();
            }
        }
    },
    
    // Connect to this event for triggers when the filter changes
    onFilterChanged: function(filter) {}

});