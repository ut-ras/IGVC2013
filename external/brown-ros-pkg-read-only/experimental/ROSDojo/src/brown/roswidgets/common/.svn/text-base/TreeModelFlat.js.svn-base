dojo.provide("roswidgets.common.TreeModelFlat");

dojo.require("rosdojo.rosdojo");

dojo.declare("roswidgets.common.TreeModelFlat", null, {
    /*
     * FlatTreeModel is a very basic implementation of dijit.tree.Model
     * 
     * It does background polling of ROS to check whether new topics or services appear
     * 
     * Results are displayed in a flat list with just a single root node
     */

    // Refresh rate in ms
    refreshRate: 10000,
    
    // Parameters; must be set by subclasses or callers
    rootLabel: "",
    
    constructor: function(args) {
        dojo.mixin(this, args);
        
        this.clear();
        
        // Begin refreshing
        this.onPoll();
        
    },
    
    // Connect to this function - the tree will call this to poll for data.  Return data to the tree with the setData function
    onPoll: function() {
        window.clearTimeout(this.refreshTimer);
        this.refreshTimer = window.setTimeout(dojo.hitch(this, "onPoll"), this.refreshRate);        
    },
    
    // Set the tree data
    setData: function(data) {
        this._processData(data);
        window.clearTimeout(this.refreshTimer);
        this.refreshTimer = window.setTimeout(dojo.hitch(this, "onPoll"), this.refreshRate);
    },
    
    uninitialize: function() {
        window.clearTimeout(this.refreshTimer);
        this.setData = function() {};
    },
    
    // Clears the list.  Doesn't prevent polling
    clear: function() {
        this.itemArray = [];
        this.itemMap = {};
        this.onChildrenChange(this.rootLabel, this.itemArray);
    },
    
    // Interface methods
    getChildren: function(parentItem, onComplete) { items = parentItem==this.rootLabel ? this.items : []; onComplete(items); },
    getIdentity: function(item) { return item; },
    getLabel: function(item) { return item; },
    getRoot: function(onItem) { onItem(this.rootLabel); },
    isItem: function(something) { return something==this.rootLabel || this.itemMap[something]; },
    mayHaveChildren: function(item) { return item==this.rootLabel; },
    fetchItemByIdentity: function(keywordArgs) { /* Not Implemented */ },
    newItem: function(args, parent, insertIndex) { /* Not implemented */ },
    pasteItem: function(childItem, oldParentItem, newParentItem, bCopy) { /* Not implemented */ },

    // Interface callbacks
    onChange: function(item) {},
    onChildrenChange: function(parent, newChildrenList) {},
    
    // Non-interface methods    
    isValid: function(item) {
        return this.itemMap[item];
    },
    
    // Non-interface callbacks    
    _processData: function(newItems) {
        var oldItemMap = this.itemMap;
        var changed = false;
        for (var i = 0; i < newItems.length; i++) {
            if (!oldItemMap[newItems[i]]) {
                changed = true;
            }
        }
        
        if (changed) {
            var newItemMap = {};
            for (var i = 0; i < newItems.length; i++) {
                newItemMap[newItems[i]] = true;
            }
            this.itemArray = newItems.sort();
            this.itemMap = newItemMap;
            this.onChildrenChange(this.rootLabel, this.itemArray);
        }
    }

});