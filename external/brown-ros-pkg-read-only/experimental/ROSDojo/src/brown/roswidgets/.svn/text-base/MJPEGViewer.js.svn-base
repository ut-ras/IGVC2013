dojo.provide("roswidgets.MJPEGViewer");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.MJPEGViewer", [ dijit._Widget ], {
    
    // Arguments
    topic: null,
    port: 8080,
    quality: 100,
    width: 320,
    height: 240,
    
    postCreate : function() {
    	dojo.addClass(this.domNode, "mjpegviewer");
    	dojo.style(this.domNode, "width", this.width+"px");
    	dojo.style(this.domNode, "height", this.height+"px");

        dojo.connect(ros, "onConnecting", this, "onConnecting");
        dojo.connect(ros, "onClose", this, "removeImg");
        
        if (ros.available()) {
            this.createImg(ros.hostname);
        }
    },
    
    onConnecting: function(url) {
        console.log("in onConnecting");
        console.log("url = " + url);
        if (url) {
            this.createImg(url);
        }
    },
    
    createImg: function(url) {
        var img = document.createElement('img');
        img.src = "http://" + url + ":" + this.port + "/stream?topic=" + this.topic + "?width="+this.width+"?height="+this.height+"?quality="+this.quality;
        img.style.width = this.width+"px";
        img.style.height = this.height+"px";
        this.domNode.appendChild(img);
        this.img = img;
    },
    
    removeImg: function() {
        this.domNode.innerHTML="";
    }

});
