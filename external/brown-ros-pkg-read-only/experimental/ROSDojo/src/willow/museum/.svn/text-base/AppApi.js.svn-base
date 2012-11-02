dojo.provide("museum.AppApi");

dojo.require("museum.AjaxApi");
dojo.require("museum.UserApi");
dojo.require("misc.URLQueryString");

dojo.declare("museum.AppApi", null, {
    /* The default user status is passed in via URL query string */
    /* Each function can be provided with callbacks, but also events are fired anyway, so
     * to interact with this widget you can either pass one-time callbacks or sign up to all events by connecting to events
     */

    // Parameters
    use_queryarg_appid: true, // Use the session ID passed in via the URL Query args and automatically log in
    
    // Internal variables
    logged_in: false,

    constructor: function(args) {
        dojo.mixin(this, args);
        if (this.use_queryarg_appid) {
            dojo.mixin(this, misc.URLQueryString().getAll());
        }
        dojo.mixin(this, args);
        
        if (!this.user) {
            throw "Tried to instantiate an App API without a User"
        }
        
        
        // Create the ajax api
        this.api = this.user.api;         
        
        dojo.connect(this.user, "onLogout", this, "clear");
        
        if (this.app_id) {
            this.getApp(this.app_id);
        }
    },
    
    save: function(name, data) {
        if (this.app_id) {
            this.saveApp(name, data, this.app_id);
        } else {
            this.createApp(name, data);
        }
    },
    
    clear: function() {
        this.app_id = null;
        this.app = null;
    },
    
    currentApp: function() {
        return this.app; 
    },
    
    getApp: function(app_id) {
        var callback = dojo.hitch(this, "callbackAppReceived");
        var errback = dojo.hitch(this, "onLoadAppFailed");
        this.api.get_app(this.user.session_key, app_id, callback, errback);
    },
    
    getApps: function() {
        var callback = dojo.hitch(this, "callbackAppsReceived");
        var errback = dojo.hitch(this, "onLoadAppsFailed");
        this.api.get_apps(this.user.session_key, callback, errback);
    },
    
    saveApp: function(appname, app, app_id) {
        if (this.user.isLoggedIn() && app_id!=null) {
            var callback = dojo.hitch(this, "callbackAppSaved");
            var errback = dojo.hitch(this, "onAppSavedFailed");
            this.api.save_app(this.user.session_key, this.app_id, appname, app, callback, errback);
        }
    },
    
    createApp: function(appname, app) {
        if (this.user.isLoggedIn()) {
            var callback = dojo.hitch(this, "callbackAppCreated");
            var errback = dojo.hitch(this, "onAppCreatedFailed");
            console.log("Creating app", this.user.session_key, appname, app);
                
            this.api.create_app(this.user.session_key, appname, app, callback, errback);
        }
    },
    
    callbackAppReceived: function(data) {
        console.log("app received", data);
        if (data && data.app_id!=null && data.app!=null) {
            this.app_id = data.app_id;
            this.app = data;
            this.onLoadApp(data.name, data.app);
        }
    },
    
    callbackAppsReceived: function(data) {
        console.log("received apps!", data);
        if (data && data.apps) {
            this.onLoadApps(data.apps);
        }
    },
    
    callbackAppSaved: function(data) {
        console.log("app saved", data);
        if (data && data.app_id!=null) {
            this.app_id = data.app_id;
            this.onAppSaved();
        }
    },
    
    callbackAppCreated: function(data) {
        console.log("app created", data);
        if (data && data.app_id!=null) {
            this.app_id = data.app_id;
            this.onAppCreated(data.app_id);
        }
    },
    
    onLoadApp: function(name, app) { console.log("onLoadApp", name); },
    onLoadAppFailed: function() { console.log("onLoadAppFailed"); },
    
    onLoadApps: function(apps) { console.log("onLoadApps", apps); },
    onLoadAppsFailed: function() { console.log("onLoadAppsFailed"); },
        
    onAppSaved: function() { console.log("onAppSaved"); },
    onAppSavedFailed: function() { console.log("onAppSavedFailed"); },
    
    onAppCreated: function() { console.log("onAppCreated"); },
    onAppCreatedFailed: function() { console.log("onAppCreatedFailed"); }
});
