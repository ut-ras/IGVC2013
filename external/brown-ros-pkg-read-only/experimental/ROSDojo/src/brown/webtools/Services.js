dojo.provide("webtools.Services");

dojo.require("dijit.layout.ContentPane");

dojo.require("webtools.Utils");

dojo.declare("webtools.Services", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "Services",
    
    // Internal variables
    service: null,
    
    postCreate: function() {
        // Set the class of the main dom node
        dojo.addClass(this.domNode, "content split services");
        
        // Create a container for the list
        this.listdiv = document.createElement('div');
        dojo.addClass(this.listdiv, "left");
        this.domNode.appendChild(this.listdiv);
                
        // Create a container for the tabs
        this.tabsdiv = document.createElement('div');
        dojo.addClass(this.tabsdiv, "right");
        this.domNode.appendChild(this.tabsdiv);
        
        // Create the clear div
        var cleardiv = document.createElement('div');
        dojo.addClass(cleardiv, "clear");
        this.domNode.appendChild(cleardiv);  
    },
    
    initialise: function() {
        // Create the list
        dojo.require("roswidgets.ServiceTree");
        this.list = new roswidgets.ServiceTree();
        
        // Attach and startup the list
        this.listdiv.appendChild(this.list.domNode);
        this.list.startup();
        
        // Create the tabs
        dojo.require("dijit.layout.TabContainer");
        this.tabs = new dijit.layout.TabContainer();
        
        // Create the content panes
        this.detailsPane = new webtools.Services.Details();

        // Add the content panes
        this.tabs.addChild(this.detailsPane);
        
        // Connect up the service changed event
        this.detailsPane.connect(this.list, "onItemSelected", "onServiceSelected");
        
        // Attach and startup the tabs
        this.tabsdiv.appendChild(this.tabs.domNode);
        this.tabs.startup();
    },
    
    onShow: function() {
        if (!this.initialised) {
            this.initialised = true;
            this.initialise();
        }
        // Only deactivate the list since we want to remember state
        this.list.activate();
        this.tabs.selectedChildWidget.onShow();
    },
    
    onHide: function() {
        this.list.deactivate();
        this.tabs.selectedChildWidget.onHide();
    }
    
});

//Abstract class for the inner panels
dojo.declare("webtools.Services.Panel", [ dijit.layout.ContentPane ], {
    
    initialised: false,
    selected: false,
    service: null,
    
    onShow: function() {
        if (!this.initialised) {
            this.initialise();
            this.initialised = true;
        }
        this.setUp();
        if (this.service) {
            this.setService(this.service);
        }
    },
    
    onHide: function() {
        this.tearDown();
    },
    
    onServiceSelected: function(service) {
        this.service = service;
        if (this.selected) {
            this.setService(service);
        }
    },
    
    // Called once, at some point before the content panel is shown
    initialise: function () { /* OVERRIDE ME */ },
    
    // Called every time the content panel is shown. Use to set up volatile elements
    setUp: function() { /* OVERRIDE ME */ },
    
    // Called every time the content panel is hidden.  Use to destroy volatile elements
    tearDown: function() { /* OVERRIDE ME */ },
    
    // This will only be called when the widget is shown
    setService: function(service) { /* OVERRIDE ME */ }
    
});

dojo.declare("webtools.Services.Details", [ webtools.Services.Panel ], {
    
    title: "Details",
    
    initialise: function() {
        dojo.require("roswidgets.ServiceDetails");
        this.servicedetails = new roswidgets.ServiceDetails();
        this.domNode.appendChild(this.servicedetails.domNode);        
    },
    
    setService: function(service) {
        if (this.servicedetails.service != service) {
            this.servicedetails.show(service);
        }
    }
    
});