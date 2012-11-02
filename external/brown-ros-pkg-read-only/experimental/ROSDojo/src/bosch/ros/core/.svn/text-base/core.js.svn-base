/**
*  NodeHandle no longer can specify its own URL, instead it proxies whatever is specified in rosdojo
*/

ros.NodeHandle = function() {}

ros.NodeHandle.prototype.subscribe = function(topic, callback) {
	ros.subscribe(topic, callback);
}

ros.ServiceClient = Class.extend({
		
	init: function(node, service) {
		this.service = service;
	},

	call: function(msg, callback) {
		ros.callService(this.service, msg, callback);
	}

});

ros.Publisher = Class.extend({

	init: function(node, topic, type) {
		this.topic = topic;
		this.type = type;
	},
	
	publish: function(message) {
		ros.publish(this.topic, this.type, dojo.toJson(message));
	}
	
})

ros.NodeHandle = Class.extend({
	
	init: function() {
		this.is_connected = true;
		this.handlers = {};
	},
	
	advertise: function(topic, type) {
		return new ros.Publisher(this, topic, type);
	},
	
	publish: function(topic, type, json) {
		ros.publish(topic, type, json);
	},
	
	// overloaded
	subscribe: function() {
		topic = arguments[0];
		callback = null;
		delay = 0;
		type = null;
		if (arguments.length==2) {
			callback = arguments[1];
		} else if (arguments.length==3) {
			type = arguments[1];
			callback = arguments[2];
		} else {
			type = arguments[1];
			callback = arguments[2];
			delay = arguments[3];
		}
		
		ros.subscribe(topic, callback, delay, type);
		
		// Save the callback for unsubscribing
		var callbacks = this.handlers[topic];
		if (!callbacks) {
			callbacks = []
		}
		callbacks.push(callback);
		this.handlers[topic] = callbacks;
	},
	
	unsubscribe: function(topic) {
		var callbacks = this.handlers[topic];
		if (callbacks) {
			for (var i = 0; i < callbacks.length; i++) {
				ros.unsubscribe(topic, callbacks[i]);
			}
		}
	},
	
	serviceClient: function(service) {
		return new ros.ServiceClient(this, service);
	},
	
	setOnClose: function(callback) {
		dojo.connect(ros, "onClose", callback);
	},
	
	setOnError: function(callback) {
		dojo.connect(ros, "onError", callback);
	},
	
	setOnOpen: function(callback) {
		dojo.connect(ros, "onOpen", callback);
	},
	
	setOnMessage: function(callback) {
		dojo.connect(ros, "onMessage", callback);
	},
	
	ok: function() {
		return this.is_connected;
	},
	
	getParam: function(param, callback) {
		console.log("CALLING A PARAMETER, NOT IMPLEMENTED");
	},
	
	waitForMessage: function(topic, timeout, callback) {
		console.log("WAIT FOR MESSAGE, NOT IMPLEMENTED");	
	},
	
	getTopics: function(callback) {
		ros.callService("/rosjs/topics", JSON.stringify([]), callback);
	},
	
	getServices: function(callback) {
		ros.callService("/rosjs/services", JSON.stringify([]), callback);
	}
	
})