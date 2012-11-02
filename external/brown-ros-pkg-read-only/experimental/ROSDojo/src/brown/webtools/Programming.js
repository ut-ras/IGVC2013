dojo.provide("webtools.Programming");

dojo.require("dijit.layout.ContentPane");

dojo.require("webtools.Utils");

dojo.declare("webtools.Programming", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "Programming",
    
    // Internal variables
    topic: null,
    
    postCreate: function() {
        dojo.addClass(this.domNode, "content split");
        
        // Set up the left side of the content pane; a camera and a console
        this.leftDiv = document.createElement('div');
        dojo.addClass(this.leftDiv, "left");
        this.domNode.appendChild(this.leftDiv);
        
        // Set up the right side of the content pane; tabs of code editors        
        this.rightDiv = document.createElement('div');
        dojo.addClass(this.rightDiv, "right");
        this.domNode.appendChild(this.rightDiv);
                
        // Add a clear div
        var clearDiv = document.createElement('div');
        dojo.addClass(clearDiv, "clear");
        this.domNode.appendChild(clearDiv);
    },
    
    initialise: function() {        
        // Create the Console
        dojo.require("cs148widgets.Console");        
        this.console = new cs148widgets.Console();
        this.leftDiv.appendChild(this.console.domNode);
        
        // Create the tabs
        dojo.require("dijit.layout.TabContainer");
        this.tabs = new dijit.layout.TabContainer();
        this.rightDiv.appendChild(this.tabs.domNode);

        dojo.require("cs148widgets.CodePanel");
        this.tabs.addChild(new dijit.layout.ContentPane({
            title: "+",
            onShow: dojo.hitch(this, "newTab")
        }));
        this.addNewPanel().codeEntry.editor.setValue("import random\n\nbumped = False" +
        		"\n\ndef moveRobot(x, z):" +
        		"\n\tmsg = {\n\t\t\"linear\": { \"x\": x, \"y\": 0, \"z\": 0 }," +
        		"\n\t\t\"angular\": { \"x\": 0, \"y\": 0, \"z\": z }\n\t}" +
        		"\n\tprint \"Moving robot %d, %d\" % (x, z)" +
        		"\n\tpublish(\"/cmd_vel\", \"geometry_msgs/Twist\", msg)" +
        		"\n\ndef processSensing(sensorPacket):\n\tglobal bumped" +
        		"\n\tbumped = sensorPacket[\"bumps_wheeldrops\"]\n\ndef enclosureEscape():" +
        		"\n\tglobal bumped\n\tif bumped:\n\t\tmoveRobot(-0.1, 0)\n\t\tsleep(0.5)" +
        		"\n\t\tmoveRobot(0, random.random()-0.5)\n\telse:\n\t\tmoveRobot(0.1, 0)" +
        		"\n\nsubscribe(\"/turtlebot_node/sensor_state\", processSensing)\n" +
        		"repeat(enclosureEscape, 200)");;
        
        this.tabs.startup();
        
    },
    
    addNewPanel: function() {
        var newTab = new cs148widgets.CodePanel();
        this.console.connect(newTab, "onPrint", "print");
        this.tabs.addChild(newTab, this.tabs.getChildren().length-1);
        return newTab;
    },
    
    resize: function() {
        this.inherited(arguments);
        var consoleHeight = dojo.contentBox(this.leftDiv).h;  
        this.console.setHeight(consoleHeight);
    },
    
    onShow: function() {
        if (!this.initialised) {
            this.initialised = true;
            this.initialise();
        }
    },
    
    newTab: function() {
        var newTab = this.addNewPanel();
        window.setTimeout(dojo.hitch(this.tabs, "selectChild", newTab), 0);        
    }
    
});
