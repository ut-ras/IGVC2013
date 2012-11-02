dojo.provide("rosdojo.bridge");
dojo.provide("rosdojo.subscription");

if (typeof WebSocket == 'undefined') dojo.global.WebSocket = MozWebSocket;

dojo.declare("rosdojo.subscriptionlist", null, {
    
    // Arguments
    topic: "", /* REQUIRED */
    bridge: null, /* REQUIRED */
    type: "", /* OPTIONAL */
    
    // Internal parameters
    subscriptions: null,
    rate: 9007199254740992,
    
    constructor: function(args) {
        this.subscriptions = [];
        dojo.mixin(this, args);
    },
    
    empty: function() {
        return this.subscriptions.length == 0;
    },
    
    add: function(callback, /*optional*/ rate, /*optional*/ type) {
        // Save the type if given
        if (type) this.type = type;
        if (!rate) rate = -1;
        
        // Add the new subscription
        this.subscriptions.push({
            "callback": callback,
            "rate": rate
        });
        
        // Resubscribe if we need faster updates than we currently get
        if (rate < this.rate) {
            this.__refresh();
        }
    },
    
    remove: function(callback) {
        var index = this.subscriptions.indexOf(callback);
        if (index >= 0) {
            var removed = this.subscriptions.splice(index, 1);
            if (removed.rate == this.rate) {
                this.__refresh();
            }
        }
    },
    
    __refresh: function() {
        // Check if we should just unsubscribe
        if (this.subscriptions.length==0) {
            this.__unsubscribe();
        } else {
            // Calculate the rate for subscription
            this.__calculateRate();
            
            // Subscribe
            this.__subscribe();
        }
    },
    
    __calculateRate: function() {
        var min = 9007199254740992;
        for (var i = 0; i < this.subscriptions.length; i++) {
            if (this.subscriptions[i].rate < min) {
                min = this.subscriptions[i].rate;
            }
        }
        this.rate = min;        
    },
    
    __subscribe: function() {
        var args = [ this.topic, this.rate ];
        if (this.type) args.push(this.type);
        var cb = dojo.hitch(this, "onSubscribed");
        this.bridge.callService("/rosbridge/subscribe", dojo.toJson(args), cb);
    },
    
    __unsubscribe: function() {
        var args = [ this.topic ];
        var cb = dojo.hitch(this, "onUnsubscribed");
        this.callService("/rosbridge/unsubscribe", dojo.toJson(args), cb);
    },
    
    onSubscribed: function(msg) {},
    onUnsubscribed: function(msg) {},
    
});

dojo.declare("rosdojo.bridge", null, {
    
    // Internal variables
    socket: null,
    seed: 0,
    
    constructor: function() {
        this.subscriptions = {};
        this.serviceCallbacks = {};
    },
    
    /*************************
     * 
     * PUBLIC METHODS
     * 
     ************************/
    
    connect: function(hostname, port) {
        // First, call the onConnecting callback
        try {
            this.onConnecting(hostname, port);
        } catch (err) {
            console.debug("Error in onConnecting callback", err);
        }
        
        // Connect to the given URL
        var url = "ws://"+hostname+":"+port;
        this.socket = new WebSocket(url);

        // Hook up the callbacks
        this.socket.onmessage = dojo.hitch(this, "__onMessage");
        this.socket.onerror = dojo.hitch(this, "__onError");
        this.socket.onopen = dojo.hitch(this, "onOpen", hostname, url);
        this.socket.onclose = dojo.hitch(this, "onClose", hostname, url);
    },
    
    disconnect: function(){
        if (this.socket) {
            this.socket.close(1000);
            this.socket = null;
        }
    },
    
    // Calls the specified named service with the provided arguments.
    // The provided callback will be called with the response
    callService: function(service, /*array*/ args, callback) {
        // Make sure we have a socket...
        if (!this.socket) {
            return;
        }
        
        // Attach an ID to the service call
        this.seed++;
        service = service+"#"+this.seed;
        
        // Set up the callbacks
        var removeFunc = dojo.hitch(this, "removeHandler", service);
        var asyncCallback = function(msg) {
            callback(msg);
            removeFunc(asyncCallback);
        };
        
        // Register the callbacks
        
        
        // Make the call
        var call = { "receiver": service, "msg": args};
        this.socket.send(dojo.toJson(call));
        
        // Return the ID that was attached to the service
        return this.seed;
    },
    
    // Publishes on the specified topic the json message provided.
    // msgType must be the string name of the type (eg. "std_msgs/String")
    publish: function(topic, msgType, msg) {
        if (this.socket) {
            typeStr.replace(/^\//,'');
            var call = { 
                "receiver": topic, 
                "type": msgType, 
                "msg": msg 
            };
            this.socket.send(dojo.toJson(call));
        }
    },
    
    
    subscribe: function(topic, callback, /*optional*/ rate, /*optional*/ msgType) {
        
    },
    
    unsubscribe: function(topic, callback) {
        
    },
    
    status: function() {
        return this.socket ? this.socket.readyState : 3;
    },
    
    available: function() {
        return this.status() < 2;
    },
    
    /************************
     * 
     * PUBLIC CALLBACKS
     * 
     ************************/
    
    onConnecting: function(hostname, url) {},
    onOpen: function(hostname, url) {},
    onClose: function(hostname, url) {},
    onMessage: function(msg) {},
    onError: function(error) {},
    
    /************************
     * 
     * PRIVATE METHODS
     * 
     ************************/
    
    __hasHandlers: function(topic) {
        return topic in this.handlers && this.handlers[topic].length > 0;
    },
    
    __addServiceCallback: function(topic, func) {
        if (!topic in this.handlers) {
            this.handlers[topic] = [];
        }
        this.handlers[topic].push(func);
    },
    
    __removeServiceCallback: function(topic, func) {
        if (topic in this.handlers) {
            var index = this.handlers[topic].indexOf(func);
            if (index>=0) this.handlers[topic].splice(index, 1);
        }
    },
    
    uninitialize: function() {
        this.disconnect();
    }
    
});