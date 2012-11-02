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

dojo.global.ros = dojo.global.ros || {}
var ros = dojo.global.ros

if (typeof WebSocket == 'undefined') {
    WebSocket = MozWebSocket;
}

ros.Connection = function(url) {
    // Initialise the socket and connection variables
    this.handlers = {};
    this.socket = new WebSocket(url);
    
    // Set the onmessage socket callback
    this.onmessage = null;
    var self = this;
    this.socket.onmessage = function(e) {
        // If somebody has set a custom onmessage callback, invoke it
        if(self.onmessage) {
            try {
                self.onmessage(e);
            } catch(err) {
                // Try to handle the error if the callback fails, but
                // no biggie
                if (ros_debug) {
                    ros_debug(err);
                }
            }
        }
        
        // Try to get the data from the JSON string.  Insecure.
        var call = ''; 
        try {
            eval('call = ' + e.data);
        } catch(err) {
            return;
        }
        
        // Call any handlers registered for the message receiver
        for (var i in self.handlers[call.receiver]) {
//            console.log("Incoming message on", call.receiver, "has length", e.data.length);
            var handler = self.handlers[call.receiver][i];
            handler(call.msg);
        }
    };
    
    // Save an array of 'magic services'.  Not sure what this is doing here.
    this.magicServices = new Array(
                    '/rosbridge/subscribe',
                    '/rosbridge/unsubscribe',
                    '/rosbridge/topics',
                    '/rosbridge/services',
                    '/rosbridge/typeStringFromTopic',
                    '/rosbridge/typeStringFromService',
                    '/rosbridge/msgClassFromTypeString',
                    '/rosbridge/reqClassFromTypeString',
                    '/rosbridge/rspClassFromTypeString',
                    '/rosbridge/classFromTopic',
                    '/rosbridge/classesFromService');
    
};

ros.Connection.prototype.callService = function(service, json, callback) {
    if (this.socket && this.handlers) {
        this.handlers[service] = new Array(callback);
        var call = '{"receiver":"' + service + '"';
        call += ',"msg":' + json + '}';
        this.socket.send(call);
    }
};

ros.Connection.prototype.publish = function(topic, typeStr, json) {
    if (this.socket) {
        typeStr.replace(/^\//,'');
        var call = '{"receiver":"' + topic + '"';
        call += ',"msg":' + json;
        call += ',"type":"' + typeStr + '"}';
        this.socket.send(call);
    }
};

ros.Connection.prototype.addHandler = function(topic, func) {
    if (this.handlers) {
        if (!(topic in this.handlers)) {
            this.handlers[topic] = new Array();
        }
        this.handlers[topic].push(func);
    }
};

ros.Connection.prototype.removeHandler = function(topic, func) {
    if (this.handlers) {
        if (topic in this.handlers) {
            var index = this.handlers[topic].indexOf(func);
            if (index!=-1) {
                this.handlers[topic].splice(index, 1);
            }
        }
    }
};

ros.Connection.prototype.hasHandlers = function(topic) {
    return this.handlers && (topic in this.handlers) && (this.handlers[topic].length > 0);
};

ros.Connection.prototype.setOnError = function(func) {
    if (this.socket) {
        this.socket.onerror = func;
    }
};

ros.Connection.prototype.setOnClose = function(func) {
    if (this.socket) {
        this.socket.onclose = func;
    }
};

ros.Connection.prototype.setOnOpen = function(func) {
    if (this.socket) {
        this.socket.onopen = func;
    }
};

ros.Connection.prototype.setOnMessage = function(func) {
    this.onmessage = func;
};

ros.Connection.prototype.disconnect = function() {
    if (this.socket) {
        this.socket.close(1000);
    }
};

//Returns the WebSocket's status, as defined in the WebSocket spec:
//CONNECTING = 0; OPEN = 1; CLOSING = 2; CLOSED = 3
ros.Connection.prototype.status = function() {
    return this.socket ? this.socket.readyState : 3;
};


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
        this.seed = 0;
        this.hostname = args.hostname;
        this.port = args.port;
        var url = "ws://"+this.hostname+":"+this.port;
        var connection = new ros.Connection(url);
        dojo.mixin(this, connection);
    },
    
    subscribe : function(topic, callback, /*optional*/ rate, /*optional*/ type) {
        //  Default rate of -1
        if (rate==null) {
            rate = -1;
        }
        
        // Only call the subscribe pseudo-service if we aren't already
        // subscribed
        var mustSubscribe = !this.hasHandlers(topic);
        
        // Add to ros handlers
        this.addHandler(topic, callback);
        
        if (mustSubscribe) {
            console.debug("Subscribing to " + topic);
            var args = [ topic, rate ];
            if (type) { args.push(type); }
            this.callService("/rosjs/subscribe", dojo.toJson(args), function() {
            });
        }
    },
    
    unsubscribe : function(topic, callback) {
        // Remove the ros handler
        if (this.removeHandler) {
            this.removeHandler(topic, callback);
        }
        
        // Unsubscribe from the topic if there are no more handlers left for it
        if (!this.hasHandlers(topic)) {
            console.debug("Unsubscribing from " + topic);
            var args = [ topic ];
            this.callService("/rosjs/unsubscribe", dojo.toJson(args), function() {
            });
        }
    },
    
    callServiceAsync: function(service, args, callback, /*optional*/ field) {
        var newservice = service+"#"+this.seed;
        this.seed++;
        var hitchedCallback = callback;
        if (field) {
            hitchedCallback = function(msg) { callback(msg[field]); };
        }
        var newcallback = this.asyncCallback(newservice, hitchedCallback);
        this.callService(newservice, args, newcallback);
    },
    
    asyncCallback: function(topic, callback) {        
        var newcallback = function() {
            callback.apply(this, arguments);
            if (ros.removeHandler) {
                ros.removeHandler(topic, newcallback);
            }
        };
        return newcallback;
    },

    uninitialize : function() {
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
        this.rates = {};
        this.topictypes = {};
        dojo.connect(this, "connect", this, "onConnecting");
        dojo.connect(this, "subscribe", this, "_saveSubscription");
        dojo.connect(this, "unsubscribe", this, "_deleteSubscription");
    },
    
    connect : function(hostname, port) {
        console.log('yoshank');
        // Disconnect any existing connection
        this.disconnect();
        console.log("JON: a");
        
        // Create and mixin the new connection object
        var connection = new rosdojo.connection({
            "hostname": hostname,
            "port": port
        });
        console.log("JON: a");
        dojo.mixin(this, connection);
        console.log("JON: a");
        
        this.setOnOpen(dojo.hitch(this, "onOpen"));
        this.setOnMessage(dojo.hitch(this, "onMessage"));
        this.setOnError(dojo.hitch(this, "onError"));
        this.setOnClose(dojo.hitch(this, "onClose"));
        console.log("JON: a");
        
        dojo.connect(this, "subscribe", this, "_saveSubscription");
        dojo.connect(this, "unsubscribe", this, "_deleteSubscription");
        console.log("JON: a");
        
    },
    disconnect : function() {},
    
    // The following are events that interested parties can connect to
    onConnecting : function(hostname, port) { console.info("Rosdojo connecting to ws://" + hostname + ":" + port); },
    onOpen : function(e) {
        console.info("Rosdojo successfully connected");
        window.setTimeout(dojo.hitch(this, "_resubscribe"), 100);
    },
    onMessage : function(e) {},
    onError : function(e) {},
    onClose : function(e) { console.info("Rosdojo disconnected"); },

    status: function() { return 3; },
    available: function() { return this.status() < 2; },
    callServiceAsync: function() {},
    
    _saveSubscription : function(topic, callback, /*optional*/ rate, /*optional*/ type) {
        if (!(topic in this.subscriptions)) {
            this.subscriptions[topic] = new Array();
        }
        this.subscriptions[topic].push(callback);
        if (rate) {
            this.rates[topic] = this.rates[topic] ? this.rates[topic] < rate ? this.rates[topic] : rate : rate; 
        }
        if (type) {
            this.topictypes[topic] = type;
        }
    },
    
    _deleteSubscription : function(topic, callback) {
        if (topic in this.subscriptions) {
            var index = this.subscriptions[topic].indexOf(callback);
            if (index != -1) {
                this.subscriptions[topic].splice(index, 1);
            }
            if (this.subscriptions[topic].length == 0) {
                delete this.subscriptions[topic];
                if (topic in this.rates) {
                    delete this.rates[topic];
                }
                if (topic in this.topictypes) {
                    delete this.topictypes[topic];
                }
            }
        }
    },
    
    _resubscribe : function() {
        var subscriptions = this.subscriptions;
        var rates = this.rates;
        var topictypes = this.topictypes;
        this.topictypes = {};
        this.rates = {};
        this.subscriptions = {};
        for (topic in subscriptions) {
            for ( var i = 0, len = subscriptions[topic].length; i < len; i++) {
                this.subscribe(topic, subscriptions[topic][i], rates[topic], topictypes[topic]);
            }
        }
    },
    
    // Queries rosbridge for the topics, calls callback passing an array of topic names
    // Optionally provide a type, and only the topics publishing that type will be returned
    topics: function(callback, /*optional*/ messageType) {
        if (messageType) {
            this.topicsByType(messageType, callback);
        } else {
            this.callServiceAsync('/rosapi/topics', "[]", callback, "topics");
        }
    },
    
    topicsByType: function(type, callback) {
        this.callServiceAsync('/rosapi/topics_for_type', dojo.toJson([ type ]), callback, "topics");
    },
    
    // Queries rosbridge for the services, calls callback passing an array of topic names
    services: function(callback) {
        this.callServiceAsync('/rosapi/services', "[]", callback, "services");
    },
    
    nodes: function(callback) {
        this.callServiceAsync('/rosapi/nodes', "[]", callback, "nodes");
    },
    
    topicType: function(topic, callback) {
        this.callServiceAsync('/rosapi/topic_type', dojo.toJson([ topic ]), callback, "type");        
    },
    
    serviceType: function(service, callback) {
        this.callServiceAsync('/rosapi/service_type', dojo.toJson([ service ]), callback, "type");        
    },
    
    typeDetails: function(type, callback) {
        this.callServiceAsync('/rosapi/message_details', dojo.toJson([ type ]), callback, "typedefs");
    },
    
    serviceRequestDetails: function(type, callback) {
        this.callServiceAsync('/rosapi/service_request_details', dojo.toJson([ type ]), callback, "typedefs");
    },
    
    serviceResponseDetails: function(type, callback) {
        this.callServiceAsync('/rosapi/service_response_details', dojo.toJson([ type ]), callback, "typedefs");
    },
    
    msgFields: function(msgtype, callback) {
        this.callServiceAsync('/rosbridge/fieldsFromTypeString', dojo.toJson([ msgtype ]), callback);
    },
    
    serviceNode: function(service, callback) {
        this.callServiceAsync('/rosapi/service_node', dojo.toJson([ service ]), callback, "node");
    },
    
    serviceHost: function(service, callback) {
        this.callServiceAsync('/rosapi/service_host', dojo.toJson([ service ]), callback, "host");
    },
    
    publishers: function(topic, callback) {
        this.callServiceAsync('/rosapi/publishers', dojo.toJson([ topic ]), callback, "publishers");
    },
    
    subscribers: function(topic, callback) {
        this.callServiceAsync('/rosapi/subscribers', dojo.toJson([ topic ]), callback, "subscribers");
    },
    
    serviceProviders: function(service, callback) {
        this.callServiceAsync('/rosapi/service_providers', dojo.toJson([ service ]), callback, "providers");
    },
    
    getParam: function(name, defaultValue, callback) {
        var param_callback = function(value) {
            callback(JSON.parse(value));
        }
        this.callServiceAsync('/rosapi/get_param', dojo.toJson([ name, dojo.toJson(defaultValue) ]), param_callback, "value");
    }

});

dojo.global.ros = dojo.mixin(new rosdojo.persistentconnection(), dojo.global.ros);