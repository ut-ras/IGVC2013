dojo.provide("roswidgets.HideableConnectionPanel");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("dijit.form.ComboBox");
dojo.require("dijit.form.Button");
dojo.require("dijit.Tooltip");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.HideableConnectionPanel", [ dijit._Widget, dijit._Templated ], {
    
    templateString : dojo.cache("roswidgets", "templates/HideableConnectionPanel.html"),
    defaultUrl: "localhost:9090", /* set this to a list of urls that you want to show up by default*/
    
    postCreate : function() {
        // Create the constituent dijit widgets
        var instructions = "Enter the URL of your rosbridge server, in the form hostname:port";
        this.urlLabel.title = instructions;
        this.dropdown = new dijit.form.ComboBox({ title: instructions }, this.dropdownAttach);
        this.connectButton = new dijit.form.Button({}, this.connectButtonAttach);
        this.disconnectButton = new dijit.form.Button({}, this.disconnectButtonAttach);
        
        // Populate the dropdown with previous URL values
        this.setDropdownURLs();
        
        // Hide the appropriate parts of the widget
        dojo.style(this.connectButton.domNode, "display", "");
        dojo.style(this.disconnectButton.domNode, "display", "none");
        
        // Connect events
        this.connect(this.dropdown, "onKeyDown", "dropdownKeyPressed");
        this.connect(this.connectButton, "onClick", "connectPressed");
        this.connect(this.disconnectButton, "onClick", "disconnectPressed");
        
        // Connect ros connection callbacks
        this.connect(ros, "onConnecting", "_onConnect");
        this.connect(ros, "onOpen", "_onOpen");
        this.connect(ros, "onClose", "_onClose");
        
        this.hide();
    },
    
    startup: function() {
        console.log("startup");
        window.setTimeout(dojo.hitch(this, "connectPressed"), 0);
    },
    
    dropdownKeyPressed: function(e) {
        if (!this.connecting && !this.dropdown._opened && e.keyCode == dojo.keys.ENTER) {
            this.dropdown._onBlur();
            this.connectButton.onClick();
        } 
    },
    
    connectPressed : function(event) {
        if (this.dropdown.value) {
            var url = this.defaultUrl;
            var i = url.indexOf(":");
            if (i > 0) {
                var host = url.slice(0, i);
                var port = url.slice(i+1, url.length);
                
                try {
                    console.log(ros, host, port);
                    ros.connect(host, port);
                } catch (e) {
                    this.showTooltip(e.message);
                    throw e;
                    return;
                }
            } else {
                this.showTooltip("Expecting a URL of the form <span class='italic'>hostname</span>:<span class='italic'>port</span>");
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
        
        this.hide();
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
        
        this.show();
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
        
        this.show();
    },

    // Shows a tooltip for 3 seconds with the message provided
    showTooltip : function(label) {
        if (this.tooltipTimer) {
            window.clearTimeout(this.tooltipTimer);
        }
        this.tooltipTimer = window.setTimeout(dojo.hitch(this, "hideTooltip"), 3000);
        dijit.showTooltip(label, this.tooltipAttachPoint, [ "after", "above", "below", "before" ], false, "");
    },
    
    // Immediately hides any tooltip
    hideTooltip : function() {
        dijit.hideTooltip(this.tooltipAttachPoint);
    },
    
    loadURLs : function() {
        if (urls == null) {
            urls = dojo.clone(this.defaultUrls);
        }
        return urls;
    },
    
    setDropdownURLs : function() {
        var urls = [this.defaultUrl];
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
    },
    
    hide: function() {
        this.hidden = true;
        console.log("hiding");
        dojo.style(this.panel, "visibility", "hidden");
    },
    
    show: function() {
        this.hidden = false;
        console.log("showing");
        dojo.style(this.panel, "visibility", "");
    },
    
    isHidden: function() {
        return this.hidden;
    }

});
