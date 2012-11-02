dojo.provide("roswidgets.common.Tree");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");
dojo.require("dijit.Tree");

dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.common.Tree", [ dijit._Widget, dijit._Templated ], {

    templateString: "<div class='list'><div dojoAttachPoint='treeAttach'></div></div>",

    rootLabel: "",
    flat: false, /*optional, if true, then the tree will be flat rather than expandable*/
        
    postCreate : function() {
        // Create an auto-updating rosbridge store
        var storeArgs = { 
            rootLabel: this.rootLabel
        };
        if (this.flat) {
            dojo.require("roswidgets.common.TreeModelFlat");
            this.store = new roswidgets.common.TreeModelFlat(storeArgs);
        } else {
            dojo.require("roswidgets.common.TreeModel");
            this.store = new roswidgets.common.TreeModel(storeArgs);
        }

        // Create the tree
        this.tree = new dijit.Tree({
            model: this.store
        }, this.treeAttach);
        
        this.tree.getIconClass = function(item, opened) {
            if (this.model.isValid(item)) { return "dijitLeaf"; }
            else if (this.model.mayHaveChildren(item)) { return opened ? "dijitFolderOpened" : "dijitFolderClosed"; }
            else { return "dijitLeaf"; }
        };
                
        this.connect(this.tree, "onClick", "_treeItemSelected");        
        this.connect(ros, "onOpen", "onPoll");
        this.connect(this.store, "onPoll", "onPoll");
    },
    
    onPoll: function() {},
    
    onDataReceived: function(data) {
        if (!this.deactivated) {
            this.store.setData(data);
        }
    },
    
    activate: function() { this.deactivated = false; this.onPoll(); },
    deactivate: function() { this.deactivated = true; this.onPoll(); },
    
    _treeItemSelected: function(item) {
        if (this.store.isValid(item)) {
            this.onItemSelected(item);
        }
    },

    // Callbacks to connect to
    onItemSelected: function(name) {}

});