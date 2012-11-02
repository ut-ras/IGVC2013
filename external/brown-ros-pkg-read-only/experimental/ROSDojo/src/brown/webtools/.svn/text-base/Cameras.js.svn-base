dojo.provide("webtools.Cameras");

dojo.require("dijit.layout.ContentPane");


dojo.require("webtools.Utils");

dojo.declare("webtools.Cameras", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "Cameras",
    
    // Internal variables
    topic: null,
    
    postCreate: function() {
        // Set the class of the main dom node
        dojo.addClass(this.domNode, "content split cameras");
        
        // Create a container for the list
        this.listdiv = document.createElement('div');
        dojo.addClass(this.listdiv, "left");
        this.domNode.appendChild(this.listdiv);
        
        // Create a container for the camera
        this.cameradiv = document.createElement('div');
        dojo.addClass(this.cameradiv, "right");
        this.domNode.appendChild(this.cameradiv);
        
        // Create the clear div
        var cleardiv = document.createElement('div');
        dojo.addClass(cleardiv, "clear");
        this.domNode.appendChild(cleardiv);
    },
    
    initialise: function() {
        // Create the list
        dojo.require("roswidgets.VideoTopicsTree");
        this.list = new roswidgets.VideoTopicsTree();
        
        // Attach and startup the list
        this.listdiv.appendChild(this.list.domNode);
        this.list.startup();
        
        // Create the camera
        dojo.require("roswidgets.Video");
        this.camera = new roswidgets.Video();
        
        // Attach the camera
        this.cameradiv.appendChild(this.camera.domNode);
        
        // Connect up the video topic changed event
        this.connect(this.list, "onItemSelected", "onVideoTopicSelected");
        
    },
    
    onVideoTopicSelected: function(topic) {
        this.topic = topic;
        this.camera.viewTopic(topic);
    },
    
    onShow: function() {
        if (!this.initialised) {
            this.initialised = true;
            this.initialise();
        }
        // Only deactivate the list since we want to remember state
        this.list.activate();
        if (this.topic) {
            this.camera.viewTopic(this.topic);
        }
    },
    
    onHide: function() {
        this.list.deactivate();
        this.camera.reset();
    }
    
});
