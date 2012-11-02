dojo.provide("roswidgets.TopicViewer");

dojo.require("dijit.form.Textarea");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.TopicViewer", [ dijit.form.Textarea ], {
    
    // Parameters
    topic: "", /* optional, the topic that this is displaying.  Will be changed by the widget as necessary */
    maxRate: 1000, /* topic update rate */
    typeHint: null, /* sometimes required by rosbridge (see rosbridge docs)*/
    
    // Internal variables
    count: 0,
    filterstring: "",
    
    postCreate : function() {
        // Twiddle the dom node a bit with default values
        dojo.addClass(this.domNode, "topicviewer");
        this.domNode.setAttribute("spellcheck", false);
        this.domNode.setAttribute("readonly", true);
        this.domNode.setAttribute("autocomplete", "off");
        
        // Set initial text
        this.displayInformation("Disconnected");

        // Subscribe to intial topic
        this.show(this.topic, this.maxRate, this.typeHint);
        
        // Connect to ros connection events
        this.connect(ros, "onOpen", "onOpen");
        this.connect(ros, "onClose", "onClose");
    },
    
    onOpen: function() {
        dojo.removeClass(this.domNode, "disconnected");
        if (this.topic) {
            this._show(this.topic, this.rate, this.typeHint);
        } else {
            this.displayInformation("Not subscribed to any topic");
        }
    },
    
    onClose: function() {
        this.count++;
        dojo.addClass(this.domNode, "disconnected");
    },
    
    show: function(topic, /*optional*/ rate, /*optional*/ typeHint) {
        if (rate==null) { rate = this.maxRate; };
        if (ros.available()) {
            this._show(topic, rate, typeHint);
        } else {
            this.topic = topic;
            this.typeHint = typeHint;
            this.rate = rate;
        }
    },
    
    _show: function(topic, rate, typeHint) {
        this.count++;
        this._rosUnsubscribe();
        this.displayInformation("...");
        if (typeHint) {
            this.topicTypeCallback(this.count, topic, rate, typeHint);
        } else {
            ros.topicType(topic, dojo.hitch(this, "topicTypeCallback", this.count, topic, rate));            
        }        
    },
    
    _rosSubscribe: function(topic, /*optional*/ rate, /*optional*/ typeHint) {
        this._rosUnsubscribe();
        if (topic) {
            this.count++;
            this.topic = topic;
            this.callback = dojo.hitch(this, "messageReceived", this.count);
            this.displayInformation("No messages received yet on topic " + this.topic);
            ros.subscribe(this.topic, this.callback, rate, typeHint);
        }
    },
    
    _rosUnsubscribe: function() {
        if (this.topic && this.callback) {
            ros.unsubscribe(this.topic, this.callback);
        }
        this.topic = null;
        this.typeHint = null;
        this.callback = null;        
    },

    topicTypeCallback: function(count, topic, rate, type) {
        if (this.count!=count) {
            // Ignore if the widget has progressed
            return;
        } else if (type) {
            // Either the topic already has a type, or it has
            // been provided for us
            this._rosSubscribe(topic, rate, type);            
        } else {
            // No type hint and no prior messages - cannot subscribe
            this.displayInformation("Topic " + topic + " is empty, cannot infer message type");
        }
    },
    
    messageReceived : function(count, msg) {
        if (count==this.count) {
        	this.msg = msg;
        	this.showMessage();
        }
    },
    
    displayInformation: function(text) {
        dojo.addClass(this.domNode, "empty");
        this.setValue(text);
    },
    
    filterChanged: function(filter) {
    	this.filterstring = filter;
    	if (this.msg) {
    	    this.showMessage();
    	}
    },
    
    showMessage: function() {
        var message = this.msg;
        
        // Remove the empty class
        dojo.removeClass(this.domNode, "empty");

        message = this.filterMessage(message, this.filterstring);        
        
        // Create the JSON string and set the value
        message = JSON.stringify(message, undefined, 2);
        this.setValue(message);
    },
    
    filterMessage: function(message, filterstring) {        
        // Pull out the substrings
        var substrings = filterstring.split(" ");
        
        // Remove any trivial ones
        for (var i = 0; i < substrings.length; i++) {
            if (substrings[i]=="") {
                substrings.splice(i, 1);
                i--;
            }
        }
        
        // Make a copy of the message with only the filtered fields
        if (substrings.length>0) {
            var newmsg = {};
            for (var i = 0; i < substrings.length; i++) {
                var substring = substrings[i];
                
                // Split the substring by '.'
                var splits = substring.split(".");
                
                this.matchr(splits, message, newmsg);
            }
            message = newmsg;
            this.pruner(message);
        } else {
            // If no filter string is provided, then we shorten long fields
            message = dojo.clone(message);
            this.shorten(message);
        }
        
        return message;
    },
    
    shorten: function(message) {
        if (message instanceof Array) {
            if (message.length > 6) {
                message.splice(5, message.length); /* only show the first 6 entries */
                message.push("(further entries hidden)");
            }
            for (var i = 0; i < message.length; i++) {
                message[i] = this.shorten(message[i]);
            }
        } else if (typeof message == 'object') {
            for (var x in message) {
                message[x] = this.shorten(message[x]);
            }
        } else if (typeof message == 'string' && message.length > 300) {
            message = message.substring(0, 300)+"... (shortened)";
        }
        return message;
    },
    
    // Removes any null fields
    pruner: function(message) {
        if (message instanceof Array) {
            for (var i = 0; i < message.length; i++) {
                if (!message[i]) {
                    message.splice(i, 1);
                    i--;
                } else {
                    this.pruner(message[i]);
                }
            }
        } else if (typeof message == 'object') {
            for (var x in message) {
                this.pruner(message[x]);
            }
        }
    },
    
    // Recursively pulls out the fields specified by the splits
    matchr: function(splits, full, partial) {
        // If there's nothing to match, copy all fields and return
        if (!splits || splits.length==0) {
            for (var x in full) {
                partial[x] = full[x];
            }
            return true;
        }
        
        // Get the next split
        var split = splits[0];
        // Remove any indexing
        if (split.indexOf("[")!=-1) {
            split = split.substring(0, split.indexOf("["));
        }

        if (full[split] instanceof Array) {
            // Match the array; arrays come first
            return this.matcharray(splits, full, partial);        
        } else if (splits.length == 1) {
            // Final split, match on field
            return this.matchfield(split, full, partial);
        } else if (!full[split]) {
            // Sub object doesn't exist, return false
            return false;
        } else {
            // Match the object
            return this.matchobject(splits, full, partial);
        }
    },
    
    // Returns true if 1 or more fields were successfully matched,
    // Also adds those fields to the partial object
    matchfield: function(field, full, partial) {
        if (field=="*") {
            for (var x in full) {
                partial[x] = full[x];
            }
            return true;
        } else {
            var success = false;
            for (var x in full) {
                if (x.indexOf(field)==0) {
                    success = true;
                    partial[x] = full[x];
                }
            }
            return success;
        }
    },
    
    matchobject: function(splits, full, partial) {
        // Remove the current split
        var split = splits[0];
        var nextsplits = splits.slice(1, splits.length);
        
        // Get the field we are copying
        var nextfull = full[split];
        
        // Create the object if it doesn't exist
        var nextpartial = partial[split];
        if (!nextpartial) {
            nextpartial = {};
        }
        
        // Now match the children
        var success = this.matchr(nextsplits, nextfull, nextpartial);
        
        // If we successfully matched, we copy in the result
        if (success) {
            partial[split] = nextpartial;
        }
        
        return success;
    },
    
    matcharray: function(splits, full, partial) {
        // Remove the current split
        var split = splits[0];
        
        // Check to see whether it's all objects, or a specific index, or a range
        var a = split.indexOf("[");
        var b = split.indexOf(":");
        var c = split.indexOf("]");
        var start = 0;
        var end = 9007199254740992; /* int.maxvalue, 2^53 */
        
        // Figure out the start and end indices
        if (a!=-1) {
            if (b!=-1) {
                if (a<b-1) {
                    start = parseInt(split.substring(a+1, b));
                }
                if (c!=-1) {
                    if (b<c-1) {
                        end = parseInt(split.substring(b+1, c));
                    }
                } else if (b!=split.length-1) {
                    end = parseInt(split.substring(b+1));
                }
            } else if (c!=-1) {
                if (a<c-1) {
                    start = parseInt(split.substring(a+1, c));
                    end = start;
                }
            } else if (a!=split.length-1){
                start = parseInt(split.substring(a+1));
                end = start;
            }
        }
        
        // Remove the array indices from the split name
        if (a!=-1) {
            splits[0] = split.substr(0, a);
        }
        
        // Perform the matching
        return this.matcharrayrange(splits, full, partial, start, end);
    },
    
    // Matches the specified range of an array, inclusive of the end index
    matcharrayrange: function(splits, full, partial, start, end) {
        // Remove the current split
        var split = splits[0];
        var nextsplits = splits.slice(1, splits.length);
        
        // Get the field we are copying
        var nextfull = full[split];
        
        // Make sure the array and dummy objects exist
        var nextpartial = partial[split];
        if (!nextpartial) {
            nextpartial = [];
        }
        if (nextsplits.length==0) {
            // Just directly copy values; this helps in the case that some might be strings
            for (var i = start; i < nextfull.length && i <= end; i++) {
                if (!nextpartial[i]) {
                    nextpartial[i] = nextfull[i];
                }
            }
            partial[split] = nextpartial;
            return true;
        } else {
            for (var i = start; i < nextfull.length && i <= end; i++) {
                if (!nextpartial[i]) {
                    nextpartial[i] = {};
                }
            }
        
            // Match on each object of the array
            var success = true;
            for (var i = start; i < nextpartial.length && i <= end; i++) {
                success &= this.matchr(nextsplits, nextfull[i], nextpartial[i]);
            }
        
            // If we successfully matched, we copy in the result
            if (success) {
                partial[split] = nextpartial;
            }
            
            return success;
        }
    },
    
    uninitialize : function() {
        this.count++;
        this._rosUnsubscribe();
    }

});