dojo.provide("roswidgets.ServiceDetails");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("rosdojo.rosdojo");

dojo.require("roswidgets.common.Utils");
dojo.require("roswidgets.common.Documentation");
dojo.require("roswidgets.common.Message");

dojo.declare("roswidgets.ServiceDetails", [ dijit._Widget, dijit._Templated ], {
        
    // Internal variables
    templateString : dojo.cache("roswidgets", "templates/ServiceDetails.html"),
    subwidgets: null,
    service: null,
    
    postCreate : function() {
    	this.subwidgets = [];
    	
    	if (this.service) {
    	    this.show(this.service);
    	}
    },
    
    show: function(service) {        
        // Clear all the props
        this.clear();
        
        // Save the service
        this.service = service;
        
        // Set the name
        this.setName(service);
        
        // Fetch some of the details
        this.fetchType(service);
        this.fetchNode(service);
        this.fetchHost(service);
    },
    
    clear: function() {
        this.propertiesAttach.innerHTML = "";
    },
    
    setName: function(name) {
        dojo.removeClass(this.nameAttach, "unknown");
        this.nameAttach.innerHTML="";
        
        var fragment = document.createDocumentFragment();

        var imgspan = document.createElement('span');
        dojo.addClass(imgspan, "srvicon");
        fragment.appendChild(imgspan);
        
        var img = document.createElement('img');
        dojo.addClass(img, "dijitIconFunction");
        imgspan.appendChild(img);
        
        fragment.appendChild(document.createTextNode(name));
        
        this.nameAttach.appendChild(fragment);
    },
    
    fetchType: function(service) {
        // Create the span and add it to the type attach
        var span = document.createElement('span');
        span.appendChild(document.createTextNode("retrieving..."));
        dojo.addClass(span, "unknown");
        this.typeAttach.innerHTML="";
        this.typeAttach.appendChild(span);
        
        // Fetch the service type
        ros.serviceType(service, dojo.hitch(this, "typeReceived", span));
    },
    
    typeReceived: function(span, type) {
        // Clear the span
        span.innerHTML = "";
        dojo.removeClass(span, "unknown");
        
        // Add the type and URL link
        var anchor = this.getTypeURL(type);
        anchor.appendChild(document.createTextNode(type));
        span.appendChild(anchor);

        // Fetch the request and response types
        this.fetchRequestType(type);
        this.fetchResponseType(type);  
    },
    
    fetchNode: function(service) {
        // Create the property span and add it to the list of properties
        var span = document.createElement('span');
        span.appendChild(document.createTextNode("retrieving..."));
        dojo.addClass(span, "unknown");
        this.addProp("Main Node", span);        
        
        // The callback to attach the node name to the span
        var callback = function(node) {
            span.innerHTML = "";
            dojo.removeClass(span, "unknown");
            span.appendChild(document.createTextNode(node));
        };
        
        // Fetch the node
        ros.serviceNode(service, callback);        
    },
    
    fetchHost: function(service) {
        // Create the property span and add it to the list of properties
        var span = document.createElement('span');
        span.appendChild(document.createTextNode("retrieving..."));
        dojo.addClass(span, "unknown");
        this.addProp("Host Machine", span);        
        
        // The callback to attach the node name to the span
        var callback = function(node) {
            span.innerHTML = "";
            dojo.removeClass(span, "unknown");
            span.appendChild(document.createTextNode(node));
        };
        
        // Fetch the node
        ros.serviceHost(service, callback);
    },
    
    fetchRequestType: function(service) {
        // Create the property span and add it to the list of properties
        var div = document.createElement('div');
        div.appendChild(document.createTextNode("retrieving..."));
        dojo.addClass(div, "unknown");
        this.addProp("Request Message", div);        
        
        // The callback to attach the node name to the span
        var subwidgets = this.subwidgets;
        var callback = function(typedefs) {
            div.innerHTML = "";
            dojo.removeClass(div, "unknown");
            if (typedefs && typedefs.length>0) {
                var widget = new roswidgets.common.Message({typedef: typedefs[0], typedefs: typedefs}, div);
                subwidgets.push(widget);
            }
        };
            
        // Fetch the response message
        ros.serviceRequestDetails(service, callback);
    },
    
    fetchResponseType: function(service) {
        // Create the property span and add it to the list of properties
        var div = document.createElement('div');
        div.appendChild(document.createTextNode("retrieving..."));
        dojo.addClass(div, "unknown");
        this.addProp("Response Message", div);        
        
        // The callback to attach the node name to the span
        var subwidgets = this.subwidgets;
        var callback = function(typedefs) {
            div.innerHTML = "";
            dojo.removeClass(div, "unknown");
            if (typedefs && typedefs.length>0) {
                var widget = new roswidgets.common.Message({typedef: typedefs[0], typedefs: typedefs}, div);
                subwidgets.push(widget);
            }
        };
            
        // Fetch the response message
        ros.serviceResponseDetails(service, callback);        
    },
    
    getTypeURL: function(type) {
        // Create a document fragment, regardless of whether we actually end up adding anything
        var fragment = document.createDocumentFragment();
        
        // Check to see if we can add a document url
        var pkg = ros.docs.packageName(type);
        var srv = ros.docs.serviceName(type);
        if (pkg && srv) {
            var anchor = document.createElement('a');
            anchor.href=ros.docs.serviceAPIURL(pkg, srv);
            anchor.target="_blank"; // Forces link to open in new tab
            anchor.title="Opens the API reference for " + type + " in a new tab";
            
            var imgspan = document.createElement('span');
            dojo.addClass(imgspan, "msgicon");
            anchor.appendChild(imgspan);
            
            var img = document.createElement('img');
            dojo.addClass(img, "dijitIconDocuments");
            imgspan.appendChild(img);
            
            fragment = anchor;
        }
        
        return fragment;
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
    
    uninitialize : function() {
        for (var i = 0; i < this.subwidgets.length; i++) {
        	this.subwidgets[i].destroy();
        }
    }

});