dojo.provide("blockbuilder.BlockControlPanel");

dojo.require("blockbuilder.Utils");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("dijit.form.Button");

dojo.declare("blockbuilder.BlockControlPanel", [ dijit._Widget, dijit._Templated ], {
    
    templateString : dojo.cache("blockbuilder", "templates/BlockControlPanel.html"),
    
    postCreate : function() {
        // Create the constituent dijit widgets
        this.startButton = new dijit.form.Button({}, this.startButtonAttach);
        this.stopButton = new dijit.form.Button({}, this.stopButtonAttach);
        this.restartButton = new dijit.form.Button({}, this.restartButtonAttach);
        
        // Hide all of the buttons
        dojo.style(this.startButton.domNode, "display", "none");
        dojo.style(this.stopButton.domNode, "display", "none");
        dojo.style(this.restartButton.domNode, "display", "none");
        
        // Connect events
        this.connect(this.startButton, "onClick", "onStart");
        this.connect(this.stopButton, "onClick", "onStop");
        this.connect(this.restartButton, "onClick", "onRestart");

    },
    
    showStart: function() {
        // Hide the other buttons
        this.hideAll();
        
        // Enable the start button (in case it had been disabled)
        if (this.startEnabled) {
            this.startButton.setDisabled(false);
        } else {
            this.startButton.setDisabled(true);            
        }

        // Set the correct label
        this.startButton.setLabel("Start");
        
        // Show the start button
        dojo.style(this.startButton.domNode, "display", "");        
    },    
    
    showStop: function() {
        // Hide the other buttons
        this.hideAll();
        
        // Enable the stop button (in case it had been disabled)
        this.stopButton.setDisabled(false);

        // Set the correct label
        this.stopButton.setLabel("Stop");

        // Show the stop button
        dojo.style(this.stopButton.domNode, "display", "");     
    },
    
    showRestart: function() {
        // Hide the other buttons
        this.hideAll();
        
        // Enable the restart button (in case it had been disabled)
        this.restartButton.setDisabled(false);

        // Set the correct label
        this.restartButton.setLabel("Restart");

        // Show the restart button
        dojo.style(this.restartButton.domNode, "display", "");     
    },
    
    hideAll: function() {
        dojo.style(this.startButton.domNode, "display", "none");
        dojo.style(this.stopButton.domNode, "display", "none");
        dojo.style(this.restartButton.domNode, "display", "none");
    },
    
    enableStart: function() {
        // Enable the start button (in case it had been disabled)
        this.startEnabled = true;
        this.startButton.setDisabled(false);
    },
    
    disableStart: function() {
        // Enable the start button (in case it had been disabled)
        this.startEnabled = false;
        this.startButton.setDisabled(true);
    },
    
    // Connect to these methods to receive notification when the buttons are pressed
    onStart: function() { 
        this.startButton.setDisabled(true); 
        this.startButton.setLabel("Starting...");
    },
    
    onStop: function() { 
        this.stopButton.setDisabled(true); 
        this.stopButton.setLabel("Stopping...");
    },
    
    onRestart: function() { 
        this.restartButton.setDisabled(true); 
    }

});