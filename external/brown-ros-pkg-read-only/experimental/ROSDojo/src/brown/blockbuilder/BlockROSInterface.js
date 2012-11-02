dojo.provide("blockbuilder.BlockROSInterface");

dojo.require("rosdojo.rosdojo");

dojo.require("dijit._Widget");

dojo.declare("blockbuilder.BlockROSInterface", null , {

    startService: "/pr2_block_builder/Start",
    stopService: "/pr2_block_builder/Stop",
    restartService: "/pr2_block_builder/Restart",
    
    statusTopic: "/pr2_block_builder/status",
    completionTopic: "/pr2_block_builder/completion",
    
    blockMessage: "pr2_block_builder_msgs/Blocks",
    statusMessage: "pr2_block_builder_msgs/Status",
    blockStatusMessage: "pr2_block_builder_msgs/BlockStatus",
    
    constructor : function() {
        // Subscribe to the status and completion topics
        ros.subscribe(this.statusTopic, dojo.hitch(this, "_onStatus"), this.statusMessage);
        ros.subscribe(this.completionTopic, dojo.hitch(this, "_onBlockStatus"), this.blockStatusMessage);
    },

    // Calls the 'start' service, passing the provided blocks
    start: function(blocks) {
        console.log("Calling start", blocks);
        ros.callService(this.startService, this.createBlocksMessage(blocks), this.nop);
    },
    
    // Calls the 'stop' service
    stop: function() {
        console.log("Calling stop");
        ros.callService(this.stopService, this.noargs, this.nop);
    },
    
    // Calls the 'restart' service
    restart: function() {
        console.log("Calling restart");
        ros.callService(this.restartService, this.noargs, this.nop);
    },
    
    // Callbacks that are called whenever status or completion messages are received
    onStatus: function(status, message) {},
    onBlockStatus: function(block, status, message) {},
    
    _onStatus: function(data) {
        var status = data.status.data;
        var message = data.message.data;
        console.log(status);
        this.onStatus(status, message);
    },
    
    _onBlockStatus: function(data) {
        var block = data.block;
        var status = data.status.data;
        var message = data.message.data;
        console.log(status);
        
        this.onBlockStatus(block, status, message);        
    },
    
    createBlocksMessage: function(blocks) {
        var message = { blocks: blocks };
        return dojo.toJson([message]);
    },
    
    noargs: "[]",
    nop: function() {}

});