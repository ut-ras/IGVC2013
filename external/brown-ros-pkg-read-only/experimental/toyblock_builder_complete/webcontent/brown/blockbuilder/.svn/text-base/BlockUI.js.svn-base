dojo.provide("blockbuilder.BlockUI");

dojo.require("blockbuilder.BlockPlacer");
dojo.require("blockbuilder.BlockVisualiser");
dojo.require("blockbuilder.BlockControlPanel");
dojo.require("blockbuilder.BlockROSInterface");
dojo.require("blockbuilder.Utils");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");


dojo.declare("blockbuilder.BlockUI", [ dijit._Widget, dijit._Templated ], {

    templateString : dojo.cache("blockbuilder", "templates/BlockUI.html"),
    total: 0,
    
    postCreate : function() {        
        // Create the api class
        this.rosapi = new blockbuilder.BlockROSInterface({});
        
        // Create the constituent dijit widgets
        this.controlPanel = new blockbuilder.BlockControlPanel({}, this.controlPanelAttach);
        this.blockPlacer = new blockbuilder.BlockPlacer({}, this.blockPlacerAttach);
        this.visualisation = new blockbuilder.BlockVisualiser({}, this.visualisationAttach);

        // Special method to handle 'onstart', the rest pipe through directly to the api
        dojo.connect(this.controlPanel, "onStart", this, "onStart");
        dojo.connect(this.controlPanel, "onStop", this.rosapi, "stop");
        dojo.connect(this.controlPanel, "onRestart", this.rosapi, "restart");
        
        // Hook up callbacks
        dojo.connect(this.rosapi, "onStatus", this, "onStatus");
        dojo.connect(this.rosapi, "onBlockStatus", this, "onBlockStatus");
        dojo.connect(ros, "onClose", this, "onDisconnected");
        dojo.connect(this.blockPlacer, "onBlockAdded", this, "onBlockAdded");
        dojo.connect(this.blockPlacer, "onBlockRemoved", this, "onBlockRemoved");
    },
    
    startup: function() {
        this.visualisation.startup();
    },
    
    // 
    onStart: function() {
        // Get the blocks and call start
        var blocks = this.blockPlacer.getBlocks();
        this.rosapi.start(blocks);
    },
    
    // Callback, called when ros disconnects
    onDisconnected: function() {
        this.controlPanel.hideAll();
        this.displayStatus("not connected");
    },
    
    onBlockAdded: function(x, y, z) {
        this.total++;
        if (this.total > 0) {
            this.controlPanel.enableStart();
        } else {
            this.controlPanel.disableStart();
        }
        this.visualisation.addBlock(x, y, z);
    },
    
    onBlockRemoved: function(x, y, z) {
        this.total--;
        this.visualisation.removeBlock(x, y, z);
    },
    
    onStatus: function(status, message) {
        // Handle the program status
        if (status=="ready") {
            this.controlPanel.showStart();
        } else if (status=="started") {
            this.controlPanel.showStop();
        } else if (status=="completed") {
            this.controlPanel.showRestart();
        } else if (status=="failed") {
            this.controlPanel.showRestart();
        } else if (status=="aborted") {
            this.controlPanel.showRestart();
        }
        this.displayStatus(status);
    },
    
    onBlockStatus: function(block, status, message) {
        // Handle the task status
        var displayStatus = status + " block (" + block.position.x + ", " + block.position.y + ", " + block.position.z + ") ";
        this.displayStatus(displayStatus);
    },
    
    displayStatus: function(status) {
        this.statusAttach.innerHTML = "";
        this.statusAttach.appendChild(document.createTextNode(status));
    }

});