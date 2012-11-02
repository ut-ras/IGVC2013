dojo.provide("museum.AjaxApi");

dojo.declare("museum.AjaxApi", null, {
	
	constructor: function(args) {
		if (!args.url) {
			throw "No AJAX URL specified";
		}
		dojo.mixin(this, args);
	},
	
	_xhr: function(function_name, arguments, callback, errback) {
	    var url = this.url + "/" + function_name + "/";
		var xhrArgs = { url: url, handleAs: "json" }
		if (arguments) xhrArgs.content = arguments;
		if (callback) xhrArgs.load = dojo.hitch(this, "_xhrLoad", callback, errback);
		if (errback) xhrArgs.error = errback;
		dojo.xhrPost(xhrArgs);
	},
	
	_xhrLoad: function(callback, errback, data, request) {
	    if (!data || !data.status || data.status=="error") {
	        errback(data, request);
	    } else {
	        callback(data, request);
	    }
	},
	
	ping: function(callback, errback) {
	    this._xhr("ping", {}, callback, errback);
	},
	
	get_user: function(session_key, callback, errback) {
        var args = {session_key: session_key};
        this._xhr("get_user", args, callback, errback);
	},
	
	get_apps: function(session_key, callback, errback) {
        var args = {session_key: session_key};
		this._xhr("get_apps", args, callback, errback);
	},
	
	get_app: function(session_key, app_id, callback, errback) {
	    var args = {session_key: session_key, app_id: app_id};
		this._xhr("get_app", args, callback, errback);
	},
	
	create_app: function(session_key, name, app, callback, errback) {
	    var args = {session_key: session_key, name: name, app: app};
		this._xhr("create_app", args, callback, errback);
	},
	
	save_app: function(session_key, app_id, appname, app, callback, errback) {
	    var args = {session_key: session_key, app_id: app_id, app: app, appname: appname};
        this._xhr("update_app", args, callback, errback);	    
	},
    
    login: function(username, password, callback, errback) {
        var args = {username: username, password: password};
        this._xhr("login", args, callback, errback);              
    },
    
    logout: function(session_key, callback, errback) {
        var args = {session_key: session_key};
        this._xhr("logout", args, callback, errback);
    },
    
    create_user: function(username, email, password, callback, errback) {
        var args = {username: username, email: email, password: password};
        this._xhr("create_user", args, callback, errback);              
    }
	
});