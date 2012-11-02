dojo.provide("brown.MJPEGViewer");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("rosdojo.Utils");

dojo.declare("brown.MJPEGViewer", [ dijit._Widget ], {
    
    // Arguments
    topic: null,
    port: 8080,
    quality: 100,
    width: 320,
    height: 240,
    
    postCreate : function() {

        dojo.connect(ros, "onConnecting", this, "onConnecting");
        dojo.connect(ros, "onClose", this, "removeImg");
        
    },
    
    onConnecting: function(url) {
        console.log("in onConnecting");
        console.log("url = " + url);
        url = url.slice(5, url.length - 5);
        console.log("url = " + url);
        if (url) {
            this.createImg(url);
        }
    },
    
    createImg: function(url) {
        var img = document.createElement('img');
        img.src = "http://" + url + ":" + this.port + "/stream?topic=" + this.topic + "?width="+this.width+"?height="+this.height+"?quality="+this.quality;
        this.domNode.appendChild(img);
    },
    
    removeImg: function() {
        this.domNode.innerHTML="";
    }

});
