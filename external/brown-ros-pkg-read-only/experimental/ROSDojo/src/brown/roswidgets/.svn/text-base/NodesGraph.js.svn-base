dojo.provide("roswidgets.NodesGraph");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

var script = document.createElement('script');
script.src = dojo.moduleUrl("roswidgets", "lib/GraphBox.js");
document.body.appendChild(script);

var link = document.createElement('link');
link.rel = "stylesheet";
link.href = dojo.moduleUrl("roswidgets", "lib/css/GraphBox.css");
document.body.appendChild(link);

dojo.declare("roswidgets.NodesGraph", dijit._Widget, {
    
    // Internal variables
    hiddenNodes: [ "/rosbridge", "/rosout" ],
    seed: 0,
    
    postCreate : function() {
        // Initialise variables
        this.nodes = {};
        
        // Set style classes
        dojo.addClass(this.domNode, "nodesgraph");
        
        // Create and attach the graph
        this.graph = new GraphBox.Graph();
        this.domNode.appendChild(this.graph.element);
    },
    
    fetchGraph: function() {
        ros.nodes(dojo.hitch(this, "_nodesDataCallback", this.seed));
    },
    
    _nodesDataCallback: function(seed, data) {
        if (seed==this.seed) {
            ros.systemState(dojo.hitch(this, "_systemDataCallback", seed, data));            
        }
    },
    
    _systemDataCallback: function(seed, nodes, data) {
        if (seed==this.seed) {
            this.updateGraph(nodes, data.published_topics, data.subscribed_topics);
        }
    },
    
    updateGraph: function(node_names, all_publishers, all_subscribers) {
        // Update the currently displayed nodes
        this.setNodes(node_names);
        
        // Update the connections between nodes
        this.setConnections(all_publishers, all_subscribers);
        
        // Update the graph
        this.graph.updateBounds();
    },
    
    setNodes: function(node_names) {
        // Make sure the hidden nodes aren't present
        this.removeHiddenNodes(node_names);
        
        // Delete any nodes that no longer exist
        for (var name in this.nodes) {
            if (node_names.indexOf(name)==-1) {
                this.graph.removeNode(this.nodes[name]);
                delete this.nodes[name];
            }
        }
        
        // Add in any new nodes
        for (var i = 0; i < node_names.length; i++) {
            var name = node_names[i];
            if (!this.nodes[name]) {
                var node = new GraphBox.Node(name);
                this.nodes[name] = node;
                this.graph.addNode(node);
            }
        }
    },
    
    setConnections: function(all_publishers, all_subscribers) {
        // Make sure the hidden nodes aren't present
        this.removeHiddenNodes(all_publishers);
        this.removeHiddenNodes(all_subscribers);
        
        // First, clear out the previous connections.
        for (var name in this.nodes) {
            this.nodes[name].inputs = [];
            this.nodes[name].outputs = [];
        }

        // Now create all of the new connections
        for (var topic in all_publishers) {
            var publishers = all_publishers[topic];
            var subscribers = all_subscribers[topic];
            
            // Only show a topic on a node if there are both publishers and subscribers
            if (publishers && subscribers && publishers.length>0 && subscribers.length>0) {
                var outputs = [];
                var inputs = [];
                
                // Add the outbound connection on all of the publisher nodes
                for (var i = 0; i < publishers.length; i++) {
                    var name = publishers[i];
                    var connection = new GraphBox.Connection(topic);
                    outputs.push(connection);
                    this.nodes[name].outputs.push(connection);
                }
                
                // Add the inbound connection on all of the subscriber nodes
                for (var i = 0; i < subscribers.length; i++) {
                    var name = subscribers[i];
                    var connection = new GraphBox.Connection(topic);
                    inputs.push(connection);
                    this.nodes[name].inputs.push(connection);
                }
                
                // Connect up the cross product of outbound and inbound
                for (var i = 0; i < outputs.length; i++) {
                    for (var j = 0; j < inputs.length; j++) {
                        outputs[i].connect(inputs[j]);
                    }
                }
            }
        }
        
        // Refresh the nodes by adding them to the graph again
        for (var name in this.nodes) {
            this.graph.addNode(this.nodes[name]);
        }
    },
    
    removeHiddenNodes: function(elems) {
        // Either elems is an array of node names, or a map from topic name to list of node names
        if (elems instanceof Array) {
            for (var i = 0; i < this.hiddenNodes.length; i++) {
                var index = elems.indexOf(this.hiddenNodes[i]);
                if (index >= 0) elems.splice(index, 1);
            }            
        } else {
            for (var x in elems) {
                this.removeHiddenNodes(elems[x]);
            }
        }
    },

});
