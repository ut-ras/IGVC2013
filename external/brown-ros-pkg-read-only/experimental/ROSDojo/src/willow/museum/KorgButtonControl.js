dojo.provide("museum.KorgButtonControl")

dojo.require("dijit.form.Button");
dojo.require("dijit.form.ComboButton");
dojo.require("dijit.form.NumberSpinner");
dojo.require("dijit.Menu");
dojo.require("dijit.MenuItem");

dojo.require("dijit._Widget");

dojo.declare("museum.KorgButtonControl", dijit._Widget, {

buttonNames : [ "Prev", "Play", "Next", "PlayAll", "Stop",
            "Rec" ],
    attachPoint : null,
    korgChanged : function() {
    },
    last_msg : {},
    button : {},
    callback : {},

    postCreate : function() {
        this.createCallbacks();
        this.createButtons();
    },

    createButtons : function() {
        this.button = {};

        for (i in this.buttonNames) {
            var name = this.buttonNames[i];
            var button = this.createButton(name);
            this.button[name] = button;
            this.domNode.appendChild(button.domNode);
        }

        this.createTimeSpinner();
        this.createModeChanger();
    },

    createButton : function(name) {
        var iconname = name + 'icon';
        var button = new dijit.form.Button({
            title : name,
            iconClass : iconname,
            showLabel : false
        });
        this.connect(button, "onClick", this.callback[name]);
        return button;
    },

    createTimeSpinner : function() {
        var p = document.createElement('p');
        p.innerText = "Time : ";
        this.timeSpinner = new dijit.form.NumberSpinner({
            value : 5.0,
            smallDelta : 0.1,
            constraints : {
                min : 0.1,
                max : 10
            },
            style : "width:75px"
        });

        p.appendChild(this.timeSpinner.domNode);
        this.domNode.appendChild(p);
    },

    createModeChanger : function() {
        var that = this;
        this.modeButton = new dijit.form.ComboButton({
            label : 'Slider Mode'
        });
        var menu = new dijit.Menu({
            style : "display:none;"
        });
        var menuitem1 = new dijit.MenuItem({
            label : "Slider Mode",
            onClick : function() {
                that.modeButton.attr('label', 'Slider Mode');
                that.setMode('Slider')
            }
        });
        menu.addChild(menuitem1);

        var menuitem2 = new dijit.MenuItem(
                {
                    label : "Manniquin Mode",
                    onClick : function() {
                        that.modeButton.attr('label',
                                'Manniquin Mode');
                        that.setMode('Manniquin')
                    }
                });
        menu.addChild(menuitem2);

        this.modeButton.attr('dropDown', menu);
        dojo.style(this.modeButton, "width", "300px");
        this.domNode.appendChild(this.modeButton.domNode);
    },

    setMode : function(mode) {
        console.log(mode);
    },

    korgMessageReceived : function(msg) {
        var offset = 18;

        for ( var i = 0; i < this.buttonNames.length; i++) {
            var idx = i + offset;
            if (msg.buttons[idx] == 1
                    && (this.last_msg == null || this.last_msg.buttons[idx] == 0)) {
                var name = this.buttonNames[i];
                this.callback[name]();
            }
        }

        this.last_msg = msg;
    },

    createCallbacks : function() {
        this.callback = {};

        this.callback["Prev"] = dojo.hitch(this,
                "onPreviousButton");
        this.callback["Play"] = dojo
                .hitch(this, "onPlayButton");
        this.callback["Next"] = dojo
                .hitch(this, "onNextButton");
        this.callback["PlayAll"] = dojo.hitch(this,
                "onRefreshButton");
        this.callback["Stop"] = dojo
                .hitch(this, "onStopButton");
        this.callback["Rec"] = dojo.hitch(this,
                "onRecordButton");

    },

    // Callback is called when the 'record pose' button is pressed. This method
    // should be overridden/connected to
    onPreviousButton : function() {
        console.log("previous");
    },
    onPlayButton : function() {
        console.log("play");
    },
    onNextButton : function() {
        console.log("next");
    },
    onRefreshButton : function() {
        console.log("refresh");
    },
    onStopButton : function() {
        console.log("stop");
    },
    onRecordButton : function() {
        console.log("record");
    }

});
