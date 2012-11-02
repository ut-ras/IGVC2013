dojo.provide("roswidgets.common.ImageViewer");

var script = document.createElement('script');
script.src = dojo.moduleUrl("roswidgets", "lib/wsview.js");
document.body.appendChild(script);

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");

dojo.declare("roswidgets.common.ImageViewer", dijit._Widget, {
    
    // Parameters
    topic: null, /* required */
    width: 360, /* optional */
    height: 240, /* optional */
    useMsgDimensions: false, /* optional; if true, the width and height will be set based on the dimensions specified in incoming messages*/
    
    // Internal Variables
    canvas: null,    
    wsview: null,
    alive: true,
    
    postCreate : function() {
        dojo.addClass(this.domNode, "imageviewer");
        
        this.canvas = document.createElement('canvas');
        dojo.addClass(this.canvas, "videofeed");
        
        this.domNode.appendChild(this.canvas);
        
        this.wsview = new WSView(this.canvas, this.width, this.height);
        
        this.setSize(this.width, this.height);
        
        this._imageSubscribe(this.topic);
    },
    
    _imageSubscribe: function(topic) {
        var callback = dojo.hitch(this, "_onImageDataReceived");
        ros.subscribe(topic, callback, 1000);
        this.doUnsubscribe = function() {
            ros.unsubscribe(topic, callback);
        };
    },
    
    _onImageDataReceived: function(img) {
        if (this.alive) {
            if (this.useMsgDimensions) {
                this.setSize(img.width, img.height);
            }
            this.wsview.display(img);
        }
    },
    
    setSize: function(w, h) {
        dojo.style(this.canvas, "width", w+"px");
        dojo.style(this.canvas, "height", h+"px");
        dojo.style(this.domNode, "width", w+"px");
        dojo.style(this.domNode, "height", h+"px");
        this.wsview.setSize(w, h);        
    },
    
    uninitialize: function() {
        this.alive = false;
        if (this.doUnsubscribe) {
            this.doUnsubscribe();
            this.doUnsubscribe = null;
        }
    }
    
});
