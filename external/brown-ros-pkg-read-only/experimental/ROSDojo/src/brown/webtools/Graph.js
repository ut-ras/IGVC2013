dojo.provide("webtools.Graph");

dojo.require("roswidgets.NodesGraph");
dojo.require("dijit.layout.ContentPane");

dojo.require("webtools.Utils");

dojo.declare("webtools.Graph", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "ROS Nodes",
    
    postCreate: function() {
        // Set the class of the main dom node
        dojo.addClass(this.domNode, "content nodes");
        dojo.style(this.domNode, "visibility", "hidden");
        
        // Create and attach the graph
        this.graph = new roswidgets.NodesGraph();
        this.domNode.appendChild(this.graph.domNode);
        
        this.connect(this.graph, "updateGraph", "makeVisible");
    },
    
    onHide: function() {
        dojo.style(this.domNode, "visibility", "hidden");
    },
    
    onShow: function() {
        this.graph.fetchGraph();        
    },
    
    makeVisible: function() {
        dojo.style(this.domNode, "visibility", "");
    }
    
});
