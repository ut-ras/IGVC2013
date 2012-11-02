dojo.provide("museum.AppDetails");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");

dojo.require("dijit.InlineEditBox");
dojo.require("dijit.form.TextBox");

dojo.require("museum.UserApi");
dojo.require("museum.AppApi");
dojo.require("museum.AppSelector");

dojo.declare("museum.AppDetails", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {
    
    templateString: dojo.cache("museum", "templates/AppDetails.html"),
    firstTime: true,
    programqueue_url: "programqueue_url",
    
    postCreate: function() {
        // Create the login dialog box
        this.dialog = new dijit.Dialog();
        
        // Queue button is hidden since we added a big bad queue button at the top
        dojo.style(this.queueButton.domNode, "display", "none");
        
        
        // Hook up callbacks to ROS connection
        this.connect(ros, "onOpen", "onROSConnected");
        this.connect(ros, "onClose", "onROSDisconnected");
        
        if (ros.available()) {
            this.onROSConnected();
        }
    },

    onROSConnected: function() {
        // Fetch the login URL from the ros params
        ros.getParam(this.programqueue_url, "", dojo.hitch(this, "onProgramQueueURLReceived"));
    },
    
    onROSDisconnected: function() {
        this.queueURL = null;
        this.queueButton.setDisabled(true);
    },
    
    onProgramQueueURLReceived: function(url) {
        this.queueURL = url;
        this.queueButton.setDisabled(false);
    },
    
    loggedIn: function(user) {
        this.user = user;
        
        this.connect(this.user, "serverOffline", "loggedOut");

        this.app = new museum.AppApi({user: user, use_queryarg_appid: this.firstTime});
        this.firstTime = false;

        this.connect(this.app, "onLoadApp", "onLoadApp");
        this.connect(this.app, "onAppCreated", "onAppSaved");
        this.connect(this.app, "onAppSaved", "onAppSaved");
        
        dojo.removeClass(this.apiButtons, "museum-server-offline");
    },
    
    loggedOut: function() {
        dojo.addClass(this.apiButtons, "museum-server-offline");
        this.user = null;
        this.app = null;
        this.nameEditor.setValue("");
    },

    saveApp: function() {
        if (!this.user || !this.app || !this.user.isLoggedIn()) {
            this.displayMessage(this.saveButton, "You need to be logged in to save your app!");
            return;
        }
        this.displayMessage(this.saveButton, "Saving...");
        var data = this.getAppData(true);
        var name = this.nameEditor.getValue();
        this.app.save(name, data);
    },
    
    newApp: function() {
        if (confirm("Do you really want to start over?")) {
            this.nameEditor.setValue("");
            if (this.app) {
                this.app.clear();
            }
            this.onNewApp();
        }
    },
    
    loadApp: function() {
        console.log("loading an app");
        this.showLoadDialog();
    },

    displayMessage: function(around, message, /*optional*/ orientation) {
        if (!orientation) {
            orientation = ["above", "below"];
        }
        dijit.Tooltip.show(message, around.domNode, orientation);
        window.setTimeout(dojo.hitch(dijit.Tooltip, "hide", around.domNode), 5000);
    },
    
    displayQueueStatus: function(message) {
        this.displayMessage(this.queueButton, message);             
    },
    
    queueApp: function() {
        if (this.nameEditor.getValue()=="") {
            this.displayMessage(this.nameEditor, "You need to give your app a name!", ["after", "below", "above", "before"]);
            return;
        }
        if (!this.queueing) {
            this.queueing = true;
            this.queueButton.setDisabled(true);
            this.displayQueueStatus("Queueing app...");
            
            username = "anonymous";
            if (this.user) {
                var username = this.user.username;
            }
            var data = this.getSequence();
            var programname = this.nameEditor.getValue();
            username = username + "__web__";
            console.log("username is", username);
            console.log("programname is", programname);
            ros.callServiceAsync('/museum/queue_program', dojo.toJson([ username, programname, {poses: data} ]), dojo.hitch(this, "onAppQueued"), "queued");
        }
    },
    
    onAppQueued: function(queued) {
        this.queueing = false;
        this.queueButton.setDisabled(false);
        console.log("app queued", queued);
        if (queued) {
            this.displayQueueStatus("Your app is queued!");
        } else {
            this.displayQueueStatus("We couldn't queue your app.  Ask somebody to help.");            
        }
    },
    
    showLoadDialog: function() {
        // Create a fresh login form each time the dialog is popped
        this.dialog.containerNode.innerHTML = "";
        var appSelector = new museum.AppSelector({ 
            app: this.app,
            appSelected: dojo.hitch(this.dialog, "hide"),
            cancel: dojo.hitch(this.dialog, "hide")
        });
        appSelector.connect(this.dialog, "onHide", "destroy");
        this.dialog.containerNode.appendChild(appSelector.domNode);
        this.dialog.show();
    },
    
    // Connect these methods!
    onNewApp: function() {
        this.nameEditor.setValue("");
    },
    onLoadApp: function(name, app) {
        this.nameEditor.setValue(name);
        this.setAppData(app);
    },
    onAppSaved: function() {
        this.displayMessage(this.saveButton, "App successfully saved");
    },
    
    // Override these methods!
    getAppData: function() { return ""; },
    getSequence: function() { return []; },
    setAppData: function(data) {}
    
});
