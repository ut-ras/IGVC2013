dojo.provide("museum.AppSelector");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");

dojo.require("dijit.form.Button");

dojo.declare("museum.AppSelector", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {
    
    templateString: dojo.cache("museum", "templates/AppSelector.html"),

    postCreate: function() {
        if (this.app) {
            this.connect(this.app, "onLoadApps", "onAppsReceived");
            this.app.getApps();
        }
    },
    
    onAppsReceived: function(apps) {
        for (var i = 0; i < apps.length; i++) {
            this.addAppButton(apps[i]);
        }        
    },
    
    addAppButton: function(app) {
        var button = new dijit.form.Button({ label: app.name });
        this.connect(button, "onClick", dojo.hitch(this, "_appSelected", app.app_id));
        var div = document.createElement('div');
        div.appendChild(button.domNode);
        this.appButtons.appendChild(div);
    },
    
    _appSelected: function(app_id) {
        console.log(app_id, "selected");
        this.app.getApp(app_id);
        this.appSelected(app_id);
    },
    
    appSelected: function(app_id) {
    },
    
    cancel: function() {
        console.log("Cancelling load app");
    }
    
});