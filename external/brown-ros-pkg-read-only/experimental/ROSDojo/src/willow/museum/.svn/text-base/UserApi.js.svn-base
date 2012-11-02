dojo.provide("museum.UserApi");

dojo.require("museum.AjaxApi");
dojo.require("misc.URLQueryString");

dojo.declare("museum.UserApi", null, {
    /* The default user status is passed in via URL query string */
    /* Each function can be provided with callbacks, but also events are fired anyway, so
     * to interact with this widget you can either pass one-time callbacks or sign up to all events by connecting to events
     */
    
    // Parameters
    use_queryarg_sessionid: true, // Use the session ID passed in via the URL Query args and automatically log in
    
    // Internal variables
    logged_in: false,

    constructor: function(args) {
        if (this.use_queryarg_sessionid) {
            dojo.mixin(this, misc.URLQueryString().getAll());
        }
        dojo.mixin(this, args);
        
        // Create the ajax api
        if (!this.url) {
            throw "Tried to instantiate the AJAX API without providing a URL";
        }
        
        this.api = new museum.AjaxApi({
            "url": this.url
        });         
        
        if (this.session_key) {
            this.checkLogin();
        }
    },
    
    checkLogin: function() {
        if (this.session_key) {
            var callback = dojo.hitch(this, "callbackUserInfo");
            var errback = dojo.hitch(this, "onLogout");
            this.api.get_user(this.session_key, callback, errback);
        } else {
            this.callbackLogout();
        }
    },
    
    isOnline: function() {
        this.api.ping(dojo.hitch(this, "serverOnline"), dojo.hitch(this, "serverOffline"));
    },
    
    isLoggedIn: function() {
        return this.session_key!=null && this.logged_in;
    },
    
    login: function(username, password, /*optional*/ user_callback, /*optional*/ user_errback) {
        this.session_key = null;
        var callback = dojo.hitch(this, "callbackLogin", user_callback);
        var errback = dojo.hitch(this, "errbackLogin", user_errback);
        this.api.login(username, password, callback, errback);
    },
    
    createUser: function(username, email, password, /*optional*/ user_callback, /*optional*/ user_errback) {
        this.session_key = null;
        var callback = dojo.hitch(this, "callbackCreate", user_callback);
        var errback = dojo.hitch(this, "errbackCreate", user_errback);
        this.api.create_user(username, email, password, callback, errback);       
    },
    
    logout: function() {
        var callback = dojo.hitch(this, "onLogout");
        this.api.logout(this.session_key, callback, callback);
        this.session_key = null;
        this.logged_in = false;
        this.callbackLogout();
    },
    
    callbackLogin: function(user_callback, data) {
        if (user_callback) {
            user_callback(data);
        }
        if (data && data.status && data.status=="success" && data.session_key!=null) {
            this.session_key = data.session_key;
            this.logged_in = true;
            this.checkLogin();
            this.onLogin();
        } else {
            this.errbackLogin(data);
        }
    },
    
    errbackLogin: function(user_errback, data) {
        if (data && data.message!=null) {
            this.onLoginFailed(data.message);
            if (user_errback) {
                user_errback(data.message);
            }
        } else {
            this.onLoginFailed();
            if (user_errback) {
                user_errback();
            }
        }
    },
    
    callbackCreate: function(user_callback, data) {
        if (user_callback) {
            user_callback(data);
        }
        if (data && data.status && data.status=="success" && data.session_key!=null) {
            this.session_key = data.session_key;
            this.logged_in = true;
            this.checkLogin();
            this.onUserCreate();
        } else {
            this.errbackCreate(data);
        }        
    },
    
    errbackCreate: function(user_errback, data) {
        if (user_errback) {
            user_errback(data);
        }
        if (data && data.message) {
            this.onUserCreateFailed(data.message);
        } else {
            this.onUserCreateFailed(data);
        }
    },
    
    callbackLogout: function() {
        this.session_key = null;
        this.logged_in = false;
        this.onLogout();
    },
    
    callbackUserInfo: function(data) {
        if (data && data.user_id!=null) {
            this.user_id = data.user_id;
            this.username = data.username;
            this.logged_in = true;
            this.onUserInfoReceived(data.username);
        } else {
            this.onLogout();
        }
    },
    
    // Callbacks to be connected to
    serverOnline: function() {},
    serverOffline: function() {},
    
    onLogin: function() {
        console.info("Logged in!");
    },
    
    onLoginFailed: function(message) {
        console.warn("Login failed:", message);
    },
    
    onUserCreate: function() {
        console.info("Created user");
    },
    
    onUserCreateFailed: function(message) {
        console.warn("User create failed", message);
    },
    
    onLogout: function() {
        console.info("Logged Out");
    },
    
    onUserInfoReceived: function(username) {
        console.info("Logged in as "+username);
    },
    
});