dojo.provide("roswidgets.Video");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");
dojo.require("roswidgets.common.ImageViewer");

dojo.declare("roswidgets.Video", [ dijit._Widget ], {

    // User-defined parameters
    topic: null, /* required */
    
    // Internal variables
    count: 0,
    rate: 3000,
    image: null,
        
    postCreate: function() { 
        dojo.addClass(this.domNode, "video");
        
        if (this.topic) {
            this.viewTopic(this.topic);
        }
    },
    
    viewTopic: function(topic) {
        // Reset anything existing
        this.reset();
        
        // Save the new topic
        this.topic = topic;
        
        // Clear the current dom node
        this.showLoadingMessage(topic);
        
        // Attach the new video
        this.attachVideo(topic);
    },
    
    reset: function() {
        // Increment the counter
        this.count++;
        
        // Destroy any existing images
        if (this.image) {
            this.image.destroy();
            this.image = null;
        }
        
        this.topic = null;        
    },
    
    showLoadingMessage: function(topic) {
        this.domNode.innerHTML = "Awaiting video stream from "+topic+"...";
        dojo.addClass(this.domNode, "loading");        
    },
    
    attachVideo: function(topic) {
        // Create the image viewer
        this.image = new roswidgets.common.ImageViewer({ topic: topic, useMsgDimensions: false });
        
        // Attach the video
        this.domNode.innerHTML = "";
        dojo.removeClass(this.domNode, "loading");
        this.domNode.appendChild(this.image.domNode);        
    },
    
    uninitialize: function() {
        this.reset();
    }
    
});