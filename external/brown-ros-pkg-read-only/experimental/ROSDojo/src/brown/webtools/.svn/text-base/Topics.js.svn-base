dojo.provide("webtools.Topics");

dojo.require("dijit.layout.ContentPane");

dojo.require("webtools.Utils");

dojo.declare("webtools.Topics", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "Topics",
    
    // Internal variables
    initialised: false,
    
    postCreate: function() {
        // Set the class of the main dom node
        dojo.addClass(this.domNode, "content split topics");
        
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
        dojo.require("roswidgets.TopicTree");
        this.list = new roswidgets.TopicTree();
        
        // Attach and startup the list
        this.listdiv.appendChild(this.list.domNode);
        this.list.startup();
        
        // Create the tabs
        dojo.require("dijit.layout.TabContainer");
        this.tabs = new dijit.layout.TabContainer();
        
        // Create the content panes
        this.detailsPane = new webtools.Topics.Details();
        this.messagesPane = new webtools.Topics.MessageStream();
        this.editorPane = new webtools.Topics.MessageEditor();
        
        // Add the content panes
        this.tabs.addChild(this.detailsPane);
        this.tabs.addChild(this.messagesPane);
        this.tabs.addChild(this.editorPane);

        // Connect the on item selected event
        this.detailsPane.connect(this.list, "onItemSelected", "onTopicSelected");
        this.messagesPane.connect(this.list, "onItemSelected", "onTopicSelected");
        this.editorPane.connect(this.list, "onItemSelected", "onTopicSelected");
        
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

// Abstract class for the inner panels
dojo.declare("webtools.Topics.Panel", [ dijit.layout.ContentPane ], {
    
    initialised: false,
    selected: false,
    topic: null,

    onShow: function() {
        if (!this.initialised) {
            this.initialise();
            this.initialised = true;
        }
        this.setUp();
        if (this.topic) {
            this.setTopic(this.topic);
        }
    },
    
    onHide: function() {
        this.tearDown();
    },
    
    onTopicSelected: function(topic) {
        this.topic = topic;
        if (this.selected) {
            this.setTopic(topic);
        }
    },
    
    // Called once, at some point before the content panel is shown
    initialise: function () { /* OVERRIDE ME */ },
    
    // Called every time the content panel is shown. Use to set up volatile elements
    setUp: function() { /* OVERRIDE ME */ },
    
    // Called every time the content panel is hidden.  Use to destroy volatile elements
    tearDown: function() { /* OVERRIDE ME */ },
    
    // This will only be called when the widget is shown
    setTopic: function(topic) { /* OVERRIDE ME */ }
    
});

dojo.declare("webtools.Topics.Details", [ webtools.Topics.Panel ], {
    
    title: "Details",
    
    initialise: function() {
        dojo.require("roswidgets.TopicDetails");
        this.topicdetails = new roswidgets.TopicDetails();
        this.domNode.appendChild(this.topicdetails.domNode);
    },
    
    setTopic: function(topic) {
        if (this.topicdetails.topic != topic) {
            this.topicdetails.show(topic);
        }
    }
    
});

dojo.declare("webtools.Topics.MessageStream", [ webtools.Topics.Panel ], {
    
    title: "Message Stream",
    
    initialise: function() {
        dojo.addClass(this.domNode, "messagestream");

        dojo.require("roswidgets.TopicFilter");
        this.filter = new roswidgets.TopicFilter();       
        this.domNode.appendChild(this.filter.domNode); 
    },
    
    setUp: function() {
        dojo.require("roswidgets.TopicViewer");
        this.viewer = new roswidgets.TopicViewer();
        this.viewer.connect(this.filter, "onFilterChanged", "filterChanged");
        this.domNode.appendChild(this.viewer.domNode);
    },
    
    tearDown: function() {
        this.viewer.destroy();
    },
    
    setTopic: function(topic) {
        this.viewer.show(topic);
    }
    
});

dojo.declare("webtools.Topics.MessageEditor", [ webtools.Topics.Panel ], {
    
    title: "Send Messages",
    
    initialise: function() {        
        dojo.require("roswidgets.MessageEditor");
        
        this.editor = new roswidgets.MessageEditor();
        this.domNode.appendChild(this.editor.domNode);
    },
    
    setTopic: function(topic) {
        this.editor.setTopic(topic);
    }
    
});