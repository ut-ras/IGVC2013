/*
 * rosdojo - Two dojo classes that wrap rosjs
 * 
 * Author: Jonathan Mace
 * 
 * Date: 1/16/12 
 *
 * rosdojo.connection is a barebones dojo wrapper for a rosjs connection.
 * rosjs methods are mixed in, so any updates to rosjs will automatically
 * apply to the rosdojo.connection class
 * 
 * rosdojo.persistentconnection is a wrapper that particularly benefits
 * ros subscriptions; subscriptions can be made prior to connection with ros.
 * On connection loss, subscriptions are reestablished upon reconnection
 * Connection can be disconnected, and a different ros connected to,
 * and subscriptions will be transferred to the new ros.
 * 
 */

dojo.provide("rosdojo.connection");
dojo.provide("rosdojo.persistentconnection");

// Normally, we would load ros.js with a call to dojo.require.
// However, there are idiosyncracies with dojo.require.
// Specifically, files loaded with dojo.require are eval'ed,
// meaning globally defined variables in the code become only functionally
// defined
// To facilitate the global declaration of ros.Connection, we must
// load ros.js the old fashioned way. Fortunately, dojo dynamically
// rewrites the script url for us, so using this route is still sufficient
// without having to rewrite ros.js
var script = document.createElement('script');
script.src = dojo.moduleUrl("rosdojo", "lib/ros.js");
document.body.appendChild(script);

dojo.declare("rosdojo.connection", null, {
    /*
     * A simple dojo widget wrapper for ros.js
     * 
     * A new ros.Connection is instantiated and mixed in, so any changes to
     * ros.js will be automatically reflected in this widget.
     * 
     * Additionally, this widget contains helper methods for subscribing and
     * unsubscribing
     */

    constructor : function(args) {
        this.url = args.url;
        var connection = new ros.Connection(args.url);
        dojo.mixin(this, connection);
    },
    
    // Returns a handle that can be used to unsubscribe from the topic using the
    // unsubscribe method.  
    // type is an optional parameter but useful if you know it
    subscribe : function(topic, callback, /*optional*/ type) {
        // For now keep throttle at default of -1. Manually subscribe for other
        // throttle values
        var throttle = -1;
        
        // Only call the subscribe pseudo-service if we aren't already
        // subscribed
        var mustSubscribe = !this.hasHandlers(topic);
        
        // Add to ros handlers
        this.addHandler(topic, callback);
        
        if (mustSubscribe) {
            console.debug("Subscribing to " + topic);
            var args = [ topic, throttle ];
            if (type) { args.push(type); }
            this.callService("/rosjs/subscribe", dojo.toJson(args), function() {
            });
        }
    },
    
    unsubscribe : function(topic, callback) {
        // Remove the ros handler
        this.removeHandler(topic, callback);
        
        // Unsubscribe from the topic if there are no more handlers left for it
        if (!this.hasHandlers(topic)) {
            console.debug("Unsubscribing from " + topic);
            var args = [ topic ];
            this.callService("/rosjs/unsubscribe", dojo.toJson(args), function() {
            });
        }
    },
    
    destroy : function() {
        this.disconnect();
    }

});

dojo.declare("rosdojo.persistentconnection", null, {
    /*
     * This widget maintains topic subscriptions across multiple different
     * rosbridge connections. Only supports being connected to one rosbridge at
     * a time though.
     */

    constructor : function() {
        this.subscriptions = {};
        this.topictypes = {};
        dojo.connect(this, "connect", this, "onConnecting");
        dojo.connect(this, "subscribe", this, "saveSubscription");
        dojo.connect(this, "unsubscribe", this, "deleteSubscription");
    },
    
    connect : function(url) {
        // Disconnect any existing connection
        this.disconnect();
        
        // Create and mixin the new connection object
        var connection = new rosdojo.connection({
            "url" : url
        });
        dojo.mixin(this, connection);
        
        this.setOnOpen(dojo.hitch(this, "onOpen"));
        this.setOnMessage(dojo.hitch(this, "onMessage"));
        this.setOnError(dojo.hitch(this, "onError"));
        this.setOnClose(dojo.hitch(this, "onClose"));
        
        dojo.connect(this, "subscribe", this, "saveSubscription");
        dojo.connect(this, "unsubscribe", this, "deleteSubscription");
        
    },
    
    saveSubscription : function(topic, callback, /*optional*/ type) {
        if (!(topic in this.subscriptions)) {
            this.subscriptions[topic] = new Array();
        }
        this.subscriptions[topic].push(callback);
        if (type) {
            this.topictypes[topic] = type;
        }
    },
    
    deleteSubscription : function(topic, callback, /*optional*/ type) {
        if (topic in this.subscriptions) {
            var index = this.subscriptions[topic].indexOf(callback);
            if (index != -1) {
                this.subscriptions[topic].splice(index, 1);
            }
            if (this.subscriptions[topic].length == 0) {
                delete this.subscriptions[topic];
            }
        }
        if (topic in this.topictypes) {
            delete this.topictypes[topic];
        }
    },
    
    resubscribe : function() {
        var subscriptions = this.subscriptions;
        var topictypes = this.topictypes;
        this.topictypes = {};
        this.subscriptions = {};
        for (topic in subscriptions) {
            for ( var i = 0, len = subscriptions[topic].length; i < len; i++) {
                this.subscribe(topic, subscriptions[topic][i], topictypes[topic]);
            }
        }
    },
    
    // The following are events that interested parties can connect to
    onConnecting : function(url) { console.info("Rosdojo connecting to " + url); },
    onOpen : function(e) {
        console.info("Rosdojo successfully connected");
        this.resubscribe();
    },
    onMessage : function(e) {},
    onError : function(e) {},
    onClose : function(e) { console.info("Rosdojo disconnected"); },
    
    // The following methods are placeholder methods, in case they are called before a connection is made
    disconnect : function() {},
    subscribe : function(topic, callback, /*optional*/ type) {},
    unsubscribe : function(topic, callback, /*optional*/ type) {},

});

dojo.global.ros = dojo.mixin(new rosdojo.persistentconnection(), dojo.global.ros);