dojo.provide("museum.LoginPanel");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");
dojo.require("dijit.Dialog");

dojo.require("museum.UserApi");
dojo.require("museum.LoginForm");
dojo.require("museum.Utils");

dojo.declare("museum.LoginPanel", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {
    
    templateString: dojo.cache("museum", "templates/LoginPanel.html"),
    appserver_url_param: "appserver_url",

    postCreate: function() {        
        // Create the login dialog box
        this.dialog = new dijit.Dialog();
        
        // Hook up callbacks to ROS connection
        this.connect(ros, "onOpen", "onROSConnected");
        this.connect(ros, "onClose", "onROSDisconnected");
        
        if (ros.available()) {
            this.onROSConnected();
        }
    },
    
    onROSConnected: function() {
        // Fetch the login URL from the ros params
        ros.getParam(this.appserver_url_param, "", dojo.hitch(this, "onAppserverURLReceived"));
    },
    
    onROSDisconnected: function() {
        console.log("ROS disconnected");
        dojo.addClass(this.domNode, "museum-server-offline");
        this.loggedOut();
        this.user = null;
    },
    
    onAppserverURLReceived: function(url) {
        console.info("Contacting App Server at URL "+url);
        if (url=="") {
            return;
        }
        
        this.user = new museum.UserApi({ url: url });
        
        // Hook up callbacks to museum user status changes
        this.connect(this.user, "onUserInfoReceived", "setUser");
        this.connect(this.user, "onLogout", "loggedOut");
        this.connect(this.user, "serverOnline", "online");
        this.connect(this.user, "serverOffline", "offline");
        
        this.user.isOnline();
        
        if (this.user.isLoggedIn()) {
            this.user.checkLogin();
        }
    },
    
    online: function() {
        console.info("App Server "+this.user.url+" is online");
        dojo.removeClass(this.domNode, "museum-server-offline");
    },
    
    offline: function() {
        console.warn("App Server "+this.user.url+" is offline.");
        dojo.addClass(this.domNode, "museum-server-offline");
    },
    
    setUser: function(username) {
        this.hideAll();
        this.usernameAttach.innerHTML="";
        this.usernameAttach.appendChild(document.createTextNode(username));
        dojo.addClass(this.domNode, "loggedin");
    },
    
    loggedOut: function() {
        this.hideAll();
        dojo.addClass(this.domNode, "loggedout");
    },
    
    hideAll: function() {
        dojo.removeClass(this.domNode, "loggedin");
        dojo.removeClass(this.domNode, "loggedout");
    },
    
    showLoginDialog: function() {
        // Create a fresh login form each time the dialog is popped
        this.dialog.containerNode.innerHTML = "";
        var loginForm = new museum.LoginForm({ 
            user: this.user,
            contentsChanged: dojo.hitch(this.dialog, "layout"),
            finished: dojo.hitch(this.dialog, "hide")
        });
        loginForm.connect(this.dialog, "onHide", "destroy");
        this.dialog.containerNode.appendChild(loginForm.domNode);
        this.dialog.show();
    },

    onLogoutClicked: function() {
        this.user.logout();
    }
    
});