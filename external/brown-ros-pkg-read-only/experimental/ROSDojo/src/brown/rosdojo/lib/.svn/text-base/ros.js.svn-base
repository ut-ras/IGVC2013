var ros = ros || {};

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
