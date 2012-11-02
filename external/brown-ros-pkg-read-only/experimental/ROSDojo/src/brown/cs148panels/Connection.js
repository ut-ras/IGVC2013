dojo.provide("cs148panels.Connection");

dojo.require("roswidgets.ConnectionPanel");
dojo.require("dijit.layout.ContentPane");
dojo.require("dijit.form.FilteringSelect");
dojo.require("dojo.store.Memory");
dojo.require("dijit.form.Button");

dojo.require("cs148panels.Utils");

/*
 * This is a content pane that displays instructions.  Set the instructions
 * variable to either a string or a domnode 
 */
dojo.declare("cs148panels.Connection", [ dijit.layout.ContentPane ], {

    title: "Connection",
    instructions: null, /* string or domnode */
    defaultUrls: null, /* array of strings */
    
    postCreate: function() {
        dojo.addClass(this.domNode, "connection");
        
        this.createInstructions();
        this.createConnectionPanel();
        this.createEnvironmentSelectDropdown();
        this.createButtons();
        
        this.connect(ros, "onOpen", "onConnected");
        this.connect(ros, "onClose", "onDisconnected");
        
        this.onDisconnected();
    },
    
    createInstructions: function() {
        var instructionsTop = document.createElement('div');
        dojo.addClass(instructionsTop, "connectioninstructionstop");
        instructionsTop.appendChild(document.createTextNode(
                "Select a server to use from the dropdown list, " +
                "or enter your own URL for an alternative server.  " +
                "Remember to use the server you have been assigned!"
        ));
        this.domNode.appendChild(instructionsTop);
    },
    
    createConnectionPanel: function() {
        if (this.defaultUrls) {
            this.connectionPanel = new roswidgets.ConnectionPanel({ defaultUrls: this.defaultUrls });
        } else {
            this.connectionPanel = new roswidgets.ConnectionPanel();
        }
        this.domNode.appendChild(this.connectionPanel.domNode);
    },
    
    createEnvironmentSelectDropdown: function() {
        this.environmentDiv = document.createElement('div');
        dojo.style(this.environmentDiv, "paddingTop", "10px");
        this.domNode.appendChild(this.environmentDiv);
        
        var label = document.createElement('div');
        label.appendChild(document.createTextNode("Select a robot environment from the dropdown.  Clicking " +
        		"'Start Environment' will launch the simulated robot and environment"));
        dojo.style(label, "paddingBottom", "10px");
        this.environmentDiv.appendChild(label);
        
        var store = new dojo.store.Memory({
            data: [
                   { name: "Simple World", id: "simple" },
                   { name: "Obstacle World", id: "obstacle" },
                   { name: "Maze World", id: "maze"}
            ]
        });
        
        
        this.select = new dijit.form.FilteringSelect({ store: store, searchAttr: "name" });
        this.environmentDiv.appendChild(this.select.domNode);
        
        this.startButton = new dijit.form.Button({ label: "Start Environment"});
        this.environmentDiv.appendChild(this.startButton.domNode);
        dojo.style(this.startButton.domNode, "paddingLeft", "5px");
        this.startButton.setDisabled(true);
        
        this.connect(this.select, "onChange", "onEnvironmentSelected");
        this.connect(this.startButton, "onClick", "onEnvironmentStarting");
        
    },
    
    createButtons: function() {

    },
    
    onConnected: function() {
        dojo.style(this.environmentDiv, "display", "");
    },
    
    onDisconnected: function() {
        dojo.style(this.environmentDiv, "display", "none");
    },
    
    onEnvironmentSelected: function() {
        this.startButton.setDisabled(false);
    },
    
    onEnvironmentStarting: function() {
        this.select.setDisabled(true);
        this.startButton.setDisabled(true);
        
        var message = "Bringing up Simple World environment...";
        
        dijit.showTooltip(message, this.startButton.domNode, [ "after", "below", "above", "before" ], false, "");    
        
        
        window.setTimeout(dojo.hitch(this, "onEnvironmentStarted"), 3000);
    },
    
    onEnvironmentStarted: function() {
        dijit.hideTooltip(this.startButton.domNode);
        
        this.stopButton = new dijit.form.Button({ label: "Stop" });
        this.restartButton = new dijit.form.Button({ label: "Restart" });
        
        this.environmentDiv.appendChild(this.stopButton.domNode);
        this.environmentDiv.appendChild(this.restartButton.domNode);

        var message = "Simple World environment started.";
        
        dijit.showTooltip(message, this.startButton.domNode, [ "below", "before", "above", "after" ], false, "");
        var sbdn = this.startButton.domNode;
        window.setTimeout(function() {
            dijit.hideTooltip(sbdn);
        }, 3000);
    }
    

});