dojo.provide("roswidgets.common.TreeModel");

dojo.require("rosdojo.rosdojo");

dojo.declare("roswidgets.common.TreeModel", null, {
    /*
     * TreeModel is a very basic implementation of dijit.tree.Model
     * 
     * It does background polling of ROS to check whether new topics or services appear
     * 
     * Results are put into an expandable tree.  Use FlatTreeModel for flat results
     */

    // Refresh rate in ms
    refreshRate: 10000,
    
    // The label for the root of the tree
    rootLabel: "", 
    
    deactivated: false,
    
    constructor: function(args) {
        dojo.mixin(this, args);
        
        this.clear();
        
        // Begin polling for data        
        window.setTimeout(dojo.hitch(this, "onPoll"), 0);
    },
    
    // Connect to this function - the tree will call this to poll for data.  Return data to the tree with the setData function
    onPoll: function() {
        window.clearTimeout(this.refreshTimer);
        this.refreshTimer = window.setTimeout(dojo.hitch(this, "onPoll"), this.refreshRate);        
    },
    
    // Set the tree data
    setData: function(data) {
        if (!this.hasdata) {
            this._processData(data);
            this.onChildrenChange(this.rootLabel, this.itemMap[this.rootLabel]);
            this.hasdata = true;
        } else {
            this._refreshElements(this._processData(data));            
        }
        window.clearTimeout(this.refreshTimer);
        this.refreshTimer = window.setTimeout(dojo.hitch(this, "onPoll"), this.refreshRate);
    },
    
    uninitialize: function() {
        window.clearTimeout(this.refreshTimer);
        this.setData = function() {};
    },
    
    // Clears the list.  Doesn't prevent polling
    clear: function() {     
        this.items = {};
        this.itemMap = {};
        this.itemMap[this.rootLabel] = [];
        this.onChildrenChange(this.rootLabel, []);
    },
    
    // Interface methods
    getChildren: function(parentItem, onComplete) {
        onComplete(this.itemMap[parentItem] || []);
    },
    
    getIdentity: function(item) { return item; },
    getLabel: function(item) { return item; },
    getRoot: function(onItem) { onItem(this.rootLabel); },
    isItem: function(something) { return this.itemMap[something] || this.items[something]; },
    mayHaveChildren: function(item) { return this.itemMap[item]; },
    fetchItemByIdentity: function(keywordArgs) { /* Not implemented */ },
    newItem: function(args, parent, insertIndex) { /* Not implemented */ },
    pasteItem: function(childItem, oldParentItem, newParentItem, bCopy) { /* Not implemented */ },

    // Interface callbacks
    onChange: function(item) {},
    onChildrenChange: function(parent, newChildrenList) {},
    
    // Non-interface methods
    isValid: function(item) {
        return this.items[item];
    },
    
    // Processes the new list of data, adding new items and removing old items.
    // Returns a list of tree elements that need to be refreshed.
    _processData: function(data) {        
        var items = this.items;
        var newItems = {};
        
        // Keep track of everything that has to be modified
        var toRefresh = {};
        
        // Add any new items
        for (var i = 0, len = data.length; i < len; i++) {
            if (!items[data[i]]) {
                dojo.mixin(toRefresh, this.addItem(data[i]));
            }
            newItems[data[i]] = true;
        }
        
        // Remove any old items
        for (var item in items) {
            if (!newItems[item]) {
                dojo.mixin(toRefresh, this.removeItem(item));
            }
        }
        
        return toRefresh;
    },
    
    _refreshElements: function(elems) {
        for (var elem in elems) {
            this.onChildrenChange(elem, this.itemMap[elem] || []);
        }
    },

    // This method adds the item with the specified name
    // It returns a map of tree elements that need refreshing
    addItem: function(name) {
        var toRefresh = {};
        
        // First, flag that we have an item by this name
        this.items[name] = true;

        // Then check to see whether there are already children of this item in the tree,
        // and so no changes will be necessary
        if (this.itemMap[name]) {
            return;
        }
        
        // Split the item by the '/' symbol.  Get all of the
        // heads of the name that terminate with a '/'.  
        // Eg. "/this/is/an/example" would be split into
        // [ "/this", "/this/is", "/this/is/an", "/this/is/an/example" ]
        var splits = this._splits(name);
        
        for (var j = splits.length-1; j > 0; j--) {
            var previous = splits[j-1];
            var next = splits[j];
            if (!this.itemMap[previous]) {
                // This is the first child of the subsplit.  Add the child and try again with the parent
                this.itemMap[previous] = [ next ];
                toRefresh[previous] = true;
            } else if (this.itemMap[previous].indexOf(next)==-1){
                // The subsplit already exists, but this is a new child
                this.itemMap[previous].push(next);
                this.itemMap[previous].sort(dojo.hitch(this, "comparator"));
                toRefresh[previous] = true;
            } else {
                // The subsplit already exists and it already has this child, so just exit
                toRefresh[next] = true;
                break;
            }
        }    
        
        return toRefresh;
    },

    // This method removes the item with the specified name, refreshing the tree as necessary
    // It returns a map of tree elements that need refreshing
    removeItem: function(name) {
        var toRefresh = {};
        
        // Sanitise
        if (!this.items[name]) {
            return;
        }
        
        // First, flag that we no longer have this item
        delete this.items[name];
        
        // Now get the splits, as in addItem
        var splits = this._splits(name);
        
        // Check that the item being removed doesn't have children.  If it does then we do nothing 
        if (this.itemMap[splits[splits.length-1]]) {
            return;
        }
        
        // Iterate over the splits in reverse, removing from the item map if necessary
        for (var j = splits.length-2; j >= 0; j--) {
            var current = splits[j];
            var next = splits[j+1];
            if (this.items[next]) {
                // One of the subsplits is an item itself, so we finish here
                toRefresh[current] = true;                
                break;
            } else if (current!=this.rootLabel && this.itemMap[current].length==1 && this.itemMap[current][0]==next) {
                // The subsplit only has this as a child, so the subsplit gets removed and we continue to the parent
                delete this.itemMap[current];
                toRefresh[current] = true;
            } else {
                // The subsplit has other children, so remove this child but then finish
                var i = this.itemMap[current].indexOf(next);
                this.itemMap[current].splice(i, 1);
                toRefresh[current] = true;
                break;
            }
        }
        
        return toRefresh;
    },
    
    _splits: function(string) {
        var partialstrings = [ string ];
        var index = string.lastIndexOf("/");
        while (index > 0) {
            string = string.slice(0, index);
            partialstrings.push(string+"/...");
            index = string.lastIndexOf("/");
        }
        partialstrings.push(this.rootLabel);
        return partialstrings.reverse();
    },
    
    comparator: function(a, b) {
        if (this.itemMap[a]) {
            if (this.itemMap[b]) {
                return a.localeCompare(b);
            } else {
                return 1;
            }
        } else if (this.itemMap[b]) {
            return -1;
        } else {
            return a.localeCompare(b);
        }
    },

});