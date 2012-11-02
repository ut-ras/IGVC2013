dojo.provide("museum.LoginForm");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");

dojo.require("dijit.form.Button");
dojo.require("dijit.form.TextBox");
dojo.require("dijit.form.ValidationTextBox");
dojo.require("dijit.Tooltip");

dojo.declare("museum.LoginForm", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {
    
    templateString: dojo.cache("museum", "templates/LoginForm.html"),

    postCreate: function() {
        // Hook up the validation function for the 'create account' username entry box
        this.create_form_username.validator = function(value) { return value!=""; };;
        
        // Validation function for the e-mail entry box
        // commented out due to museum regs
//        var emailRegex = /^(([^<>()[\]\\.,;:\s@\"]+(\.[^<>()[\]\\.,;:\s@\"]+)*)|(\".+\"))@((\[[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\])|(([a-zA-Z\-0-9]+\.)+[a-zA-Z]{2,}))$/;
//        this.create_form_email.validator = function(value) { return emailRegex.test(value); }
        this.create_form_email.validator = function(value) { return true; };;
        
        // First password field validator is just 'not empty'
        this.create_form_pw1.validator = function(value) { return value!=""; };
        
        // Second password field validator is 'equals first password field'
        var pw1 = this.create_form_pw1;
        this.create_form_pw2.validator = function(value) { return value==pw1.getValue(); };
        this.create_form_pw2.connect(pw1, "validate", "validate");
    },
    
    validateCreateForm: function() {
        var valid = this.create_form_username.isValid() &&
                    this.create_form_email.isValid() &&
                    this.create_form_pw1.isValid() &&
                    this.create_form_pw2.isValid();
        this.createButton.setDisabled(!valid);
    },
    
    hideAll: function() {
        dojo.removeClass(this.domNode, "intro");
        dojo.removeClass(this.domNode, "create");
        dojo.removeClass(this.domNode, "login");
        this.hideAnyMessages();
    },
    
    enableCreate: function() {
        this.hideAll();
        dojo.addClass(this.domNode, "create");
        this.contentsChanged();
    },
    
    enableLogin: function() {
        this.hideAll();
        dojo.addClass(this.domNode, "login");
        this.contentsChanged();
    },
    
    enableIntro: function() {
        this.hideAll();
        dojo.addClass(this.domNode, "intro");  
        this.contentsChanged();      
    },
    
    onCreate: function() {
        // Get the user account values
        var username = this.create_form_username.getValue();
        var email = this.create_form_email.getValue();
        var password = this.create_form_pw1.getValue();
        
        // Create the callbacks
        var callback = dojo.hitch(this, "onCreateSuccess");
        var errback = dojo.hitch(this, "onCreateFailed");
        
        this.user.createUser(username, email, password, callback, errback);
    },
    
    onCreateSuccess: function() {
        alert("Account successfully created!");
        this.finished();
    },
    
    onCreateFailed: function(message) {
        if (!message) {
            message = "Something went wrong and we couldn't create your account.  Feel free to try again.";
        }
        this.displayMessage(this.create_form_username, message);
    },
    
    onLogin: function() {
        // Get the login values
        var username = this.login_form_username.getValue();
        var password = this.login_form_password.getValue();
        
        // Create the callbacks
        var callback = dojo.hitch(this, "onLoginSuccess");
        var errback = dojo.hitch(this, "onLoginFailed");
        
        this.user.login(username, password, callback, errback);        
    },
    
    onLoginSuccess: function() {
        this.finished();
    },
    
    onLoginFailed: function(message) {
        if (!message) {
            message = "Something went wrong and we couldn't log you in.  Feel free to try again.";
        }
        this.displayMessage(this.login_form_username, message);
    },
    
    displayMessage: function(around, message) {
        dijit.Tooltip.show(message, around.domNode);
        window.setTimeout(dojo.hitch(dijit.Tooltip, "hide", around.domNode), 5000);
    },
    
    hideAnyMessages: function() {
        dijit.Tooltip.hide(this.create_form_username.domNode);
        dijit.Tooltip.hide(this.login_form_username.domNode);
    },
    
    // A callback for resizing purposes
    contentsChanged: function() {},
    
    // A callback to indicate the form is done
    finished: function() {},
    
    uninitialize: function() {
        this.hideAnyMessages();
    }
    
});