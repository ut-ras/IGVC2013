dojo.provide("cs148panels.EnclosureEscape");

dojo.require("dijit.layout.ContentPane");
dojo.require("dijit.layout.TabContainer");

dojo.require("cs148widgets.CodePanel");
dojo.require("cs148widgets.Console");
dojo.require("roswidgets.MJPEGViewer");

dojo.require("cs148panels.Utils");

dojo.declare("cs148panels.EnclosureEscape", [ dijit.layout.ContentPane ], {

    title: "Enclosure Escape",
    instructions: null, /* string or domnode */
    
    postCreate: function() {
        dojo.addClass(this.domNode, "content split");
        
        // Set up the left side of the content pane; a camera and a console
        this.leftDiv = document.createElement('div');
        dojo.addClass(this.leftDiv, "left");
        this.domNode.appendChild(this.leftDiv);

        this.firstPersonCamera = new roswidgets.MJPEGViewer({ topic: "/camera/image_raw" });
        this.leftDiv.appendChild(this.firstPersonCamera.domNode);

        this.console = new cs148widgets.Console();
        this.leftDiv.appendChild(this.console.domNode);
        
        // Set up the right side of the content pane; tabs of code editors        
        this.rightDiv = document.createElement('div');
        dojo.addClass(this.rightDiv, "right");
        this.domNode.appendChild(this.rightDiv);
        
        this.tabs = new dijit.layout.TabContainer();
        this.rightDiv.appendChild(this.tabs.domNode);
        
        this.tabs.addChild(new dijit.layout.ContentPane({
            title: "+",
            onShow: dojo.hitch(this, "newTab")
        }));
        this.addNewPanel();
                
        // Add a clear div
        var clearDiv = document.createElement('div');
        dojo.addClass(clearDiv, "clear");
        this.domNode.appendChild(clearDiv);
    },
    
    startup: function() {
    	this.tabs.startup();
    },
    
    resize: function() {
        this.inherited(arguments);

        // First, stretch the camera to take up the allowed width
        dojo.style(this.firstPersonCamera.domNode, "width", "100%");
        dojo.style(this.firstPersonCamera.domNode, "height", "");
        if (ros.available()) {
            dojo.style(this.firstPersonCamera.domNode.firstChild, "height", "");
            dojo.style(this.firstPersonCamera.domNode.firstChild, "width", "100%");
        }
        
        // Then fill up the remaining space with the console
        var cameraHeight = this.firstPersonCamera.domNode.clientHeight;
        var consoleHeight = dojo.contentBox(this.leftDiv).h - cameraHeight;  
        this.console.setHeight(consoleHeight);
    },
    
    addNewPanel: function() {
        var newTab = new cs148widgets.CodePanel();
        this.console.connect(newTab, "onPrint", "print");
        this.tabs.addChild(newTab, this.tabs.getChildren().length-1);
        return newTab;
    },
    
    newTab: function() {
    	var newTab = this.addNewPanel();
        window.setTimeout(dojo.hitch(this.tabs, "selectChild", newTab), 0);        
    }
    
});