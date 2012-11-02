dojo.provide("rosdojo.ConnectionPanel");

dojo.require("dojo.cookie");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("dijit.form.ComboBox");
dojo.require("dijit.form.Button");
dojo.require("dijit.Tooltip");

dojo.require("rosdojo.rosdojo");
dojo.require("rosdojo.Utils");

dojo.declare("rosdojo.ConnectionPanel", [ dijit._Widget, dijit._Templated ], {
    
    templateString : dojo.cache("rosdojo", "templates/ConnectionPanel.html"),
    
    postCreate : function() {
        // Create the constituent dijit widgets
        this.dropdown = new dijit.form.ComboBox({}, this.dropdownAttach);
        this.connectButton = new dijit.form.Button({}, this.connectButtonAttach);
        this.disconnectButton = new dijit.form.Button({}, this.disconnectButtonAttach);
        
        // Populate the dropdown with previous URL values
        this.setDropdownURLs(this.loadURLs());
        
        // Hide the appropriate parts of the widget
        dojo.style(this.connectButton.domNode, "display", "");
        dojo.style(this.disconnectButton.domNode, "display", "none");
        
        // Connect events
        this.connect(this.connectButton, "onClick", "connectPressed");
        this.connect(this.disconnectButton, "onClick", "disconnectPressed");
        
        // Connect ros connection callbacks
        this.connect(ros, "onConnecting", "_onConnect");
        this.connect(ros, "onOpen", "_onOpen");
        this.connect(ros, "onClose", "_onClose");
    },
    
    connectPressed : function(event) {
        if (this.dropdown.value) {
            var url = this.dropdown.value;
            try {
                ros.connect("ws://" + url);
            } catch (e) {
                this.showTooltip(e.message);
                return;
            }
            try {
                this.saveURL(url);
                this.setDropdownURLs();
            } catch (e) {
                // Do nothing - Error thrown if the object already exists in the
                // store
            }
        }
    },
    
    disconnectPressed : function(event) {
        ros.disconnect();
    },
    
    onConnecting : function() {
        // Hide relevant portions of the widget
        this.dropdown.setDisabled(true);
        this.connectButton.setDisabled(true);
        this.connectButton.setLabel("Connecting...");
    },
    
    onConnected : function() {
        // Show the relevant portions of the widget
        dojo.style(this.connectButton.domNode, "display", "none");
        dojo.style(this.disconnectButton.domNode, "display", "");
        
        // Show the 'connected' tooltip
        var label = "Connected to " + this.dropdown.value;
        this.showTooltip(label);
    },
    
    onDisconnected : function() {
        // Show and hide the relevant portions of the widget
        this.dropdown.setDisabled(false);
        this.connectButton.setDisabled(false);
        this.connectButton.setLabel("Connect");
        dojo.style(this.connectButton.domNode, "display", "");
        dojo.style(this.disconnectButton.domNode, "display", "none");
        
        // Show the 'disconnected' tooltip
        var label = "Disconnected from " + this.dropdown.value;
        this.showTooltip(label);
    },
    
    onUnableToConnect : function() {
        // Show and hide the relevant portions of the widget
        this.dropdown.setDisabled(false);
        this.connectButton.setDisabled(false);
        this.connectButton.setLabel("Connect");
        dojo.style(this.connectButton.domNode, "display", "");
        dojo.style(this.disconnectButton.domNode, "display", "none");
        
        // Show the 'unable to connect' tooltip
        var label = "Unable to connect to " + this.dropdown.value;
        this.showTooltip(label);
    },
    
    // Shows a tooltip for 3 seconds with the message provided
    showTooltip : function(label) {
        if (this.tooltipTimer) {
            window.clearTimeout(this.tooltipTimer);
        }
        dijit.showTooltip(label, this.buttons, [ "after", "above", "below", "before" ], false, "");
        this.tooltipTimer = window.setTimeout(dojo.hitch(this, "hideTooltip"), 3000);
    },
    
    // Immediately hides any tooltip
    hideTooltip : function() {
        dijit.hideTooltip(this.buttons);
    },
    
    saveURL : function(url) {
        var urls = this.loadURLs();
        
        // First, remove the url from the list if it's already in it
        var i = urls.indexOf(url);
        if (i >= 0) {
            urls.splice(i, 1);
        }
        
        // Then add the url to the start of the list
        urls.splice(0, 0, url);
        
        // Prune the list if it's longer than 5
        if (urls.length > 5) {
            urls.splice(urls.length - 2, 1);
        }
        
        // Finally save the url list as a cookie
        dojo.cookie("rosbridge_connection_urls", dojo.toJson(urls), {});
    },
    
    loadURLs : function() {
        var urls = dojo.fromJson(dojo.cookie("rosbridge_connection_urls"));
        if (urls == null) {
            urls = [ "localhost:9090" ];
        }
        return urls;
    },
    
    setDropdownURLs : function() {
        var urls = this.loadURLs();
        var data = [];
        for ( var i = 0, len = urls.length; i < len; i++) {
            var url = urls[i];
            data.push({
                name : url,
                id : url,
                value : url
            });
        }
        this.dropdown.store.setData(data);
        this.dropdown.setValue(urls[0]);
    },
    
    _onConnect : function() {
        this.connecting = true;
        this.onConnecting();
    },
    
    _onOpen : function() {
        this.connecting = false;
        this.onConnected();
    },
    
    _onClose : function() {
        if (this.connecting == true) {
            this.onUnableToConnect();
        } else {
            this.onDisconnected();
        }
        this.connecting = false;
    }

});
