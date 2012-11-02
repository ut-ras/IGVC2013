dojo.provide("roswidgets.MJPEGViewer2");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.MJPEGViewer2", [ dijit._Widget ], {
    
    // Arguments
    topic: null,
    url : null,
    port: 8080,
    quality: 100,
    width: 320,
    height: 240,
    
    postCreate : function() {
    	dojo.addClass(this.domNode, "mjpegviewer");
    	dojo.style(this.domNode, "width", this.width+"px");
    	dojo.style(this.domNode, "height", this.height+"px");

    },

    startup : function() {
      this.createImg(url);
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
