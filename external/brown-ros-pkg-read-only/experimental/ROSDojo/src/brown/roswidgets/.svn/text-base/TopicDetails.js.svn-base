dojo.provide("roswidgets.TopicDetails");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("rosdojo.rosdojo");

dojo.require("roswidgets.common.Utils");
dojo.require("roswidgets.common.Documentation");
dojo.require("roswidgets.common.Message");

dojo.require("roswidgets.common.ImageViewer");

dojo.declare("roswidgets.TopicDetails", [ dijit._Widget, dijit._Templated ], {
        
    // Internal variables
    templateString : dojo.cache("roswidgets", "templates/TopicDetails.html"),
    count: 0,
    subwidgets: null,
    topic: null,
    
    postCreate : function() {
    	this.subwidgets = [];
    },
    
    show: function(topic) {
        // We're showing a new topic now.  Incrementing the ID will cause
        // any lingering callbacks from the previous topic to gracefully fail.
        this.id++;
        
        // Clear all the props
        this.clear();
        
        // Save the topic
        this.topic = topic;
        
        // Set the name and topic
        this.setName(topic);
        
        // Set the type to 'Fetching...' and call callback
        this.setType("Retrieving...", true);
        ros.topicType(topic, dojo.hitch(this, "topicTypeCallback", this.count, topic));
    },
    
    clear: function() {
        this.topic = null,
        this.propertiesAttach.innerHTML = "";
        if (this.image) {
            this.image.destroy();
            this.image = null;
        }
    },
    
    setName: function(name) {
        dojo.removeClass(this.nameAttach, "unknown");
        this.nameAttach.innerHTML="";
        
        var fragment = document.createDocumentFragment();

        var imgspan = document.createElement('span');
        dojo.addClass(imgspan, "topicicon");
        fragment.appendChild(imgspan);
        
        var img = document.createElement('img');
        dojo.addClass(img, "dijitIconConnector");
        imgspan.appendChild(img);
        
        fragment.appendChild(document.createTextNode(name));
        
        this.nameAttach.appendChild(fragment);
    },
    
    setType: function(type, greyout) {
        if (greyout) {
            dojo.addClass(this.typeAttach, "unknown");
        } else {
            dojo.removeClass(this.typeAttach, "unknown");
        }
        this.typeAttach.innerHTML="";
        
        // Add the basic text
        var fragment = document.createDocumentFragment();
        fragment.appendChild(document.createTextNode(type));
        
        // Check to see if we can add a document url
        var pkg = ros.docs.packageName(type);
        var msg = ros.docs.messageName(type);
        if (pkg && msg) {
            var anchor = document.createElement('a');
            dojo.addClass(anchor);
            anchor.href=ros.docs.messageAPIURL(pkg, msg);
            anchor.target="_blank"; // Forces link to open in new tab
            anchor.title="Opens the API reference for " + type + " in a new tab";
            
            var imgspan = document.createElement('span');
            dojo.addClass(imgspan, "msgicon");
            anchor.appendChild(imgspan);
            
            var img = document.createElement('img');
            dojo.addClass(img, "dijitIconDocuments");
            imgspan.appendChild(img);
            
            anchor.appendChild(fragment);
            
            fragment = anchor;
        }
        
        this.typeAttach.appendChild(fragment);
    },
    
    addTextProp: function(propName, propValue) {
        this.addProp(propName, document.createTextNode(propValue));
    },
    
    addProp: function(propName, domElement) {
        var row = document.createElement('div');
        dojo.addClass(row, "propRow");
        
        var name = document.createElement('div');
        dojo.addClass(name, "propName");
        name.appendChild(document.createTextNode(propName+":"));
        row.appendChild(name);
        
        var value = document.createElement('div');
        dojo.addClass(value, "propValue");
        value.appendChild(domElement);
        row.appendChild(value);
        
        var clear = document.createElement('div');
        dojo.addClass(clear, "clear");
        row.appendChild(clear);
        
        this.propertiesAttach.appendChild(row);
    },
    
    topicTypeCallback: function(count, topic, data) {
        if (this.count!=count) {
            return;
        }
        if (data==null) {
            // Set topic type to 'Unknown'
            this.setType("Unknown", true);
        } else {
            // Set topic type as appropriate
            this.setType(data, false);
            
            // Add the message fields
            this.addMessageType(data);
            
            // Add an image if possible
            if (data=="sensor_msgs/Image") {
                this.addImage(topic);
            }
        }
    },
    
    typeDefCallback: function(count, attach, typedefs) {
        if (this.count!=count) {
            return;
        }
        if (typedefs && typedefs.length>0) {
            var typedef = typedefs[0];
            this.subwidgets.push(new roswidgets.common.Message({ typedef: typedef, typedefs: typedefs }, attach));
        }
    },
    
    addMessageType: function(type) {
        // Create a div to attach the message to
        var messageDiv = document.createElement('div');
        
        // Add the empty div for now, populate on callback
        this.addProp("Message Fields", messageDiv);
        
        ros.typeDetails(type, dojo.hitch(this, "typeDefCallback", this.count, messageDiv));
    },
    
    addImage: function(topic) {   
        var attach = document.createElement('div');
        
        this.image = new roswidgets.common.ImageViewer({ topic: topic }, attach);
        
        this.addProp("Image", attach);
    },
    
    uninitialize : function() {
        this.count++;
        if (this.image) {
            this.image.destroy();
            this.image = null;
        }
        for (var i = 0; i < this.subwidgets.length; i++) {
        	this.subwidgets[i].destroy();
        }
    }

});