dojo.provide("museum.KorgSlidersControl");

dojo.require("museum.KorgSlider");
dojo.require("dijit._Widget");

dojo.declare("museum.KorgSlidersControl", dijit._Widget, {
    // Internal variables
    sliderList : [ "TILT", "PAN", "LIFT", "UPPER", "ELBOW", "FOREARM",
            "WRIST", "GRIP", "TORSO" ],
    last_msg : {},
    attachPoint : null,
    controller : null,
    korgChanged : function() {
    },
    slider : {},

    postCreate : function() {
        this.createSliders();
    },

    createSliders : function() {
        this.slider = {};

        for (i in this.sliderList) {
            var name = this.sliderList[i];
            var slider = this.createSlider(name);
            this.slider[name] = slider;
            this.domNode.appendChild(slider.domNode);
        }
    },

    createSlider : function(name) {
        var slider = new museum.KorgSlider({
            label : name
        });
        this.connect(slider, "onSliderMoved", "onVirtualKorgChanged");
        return slider;
    },

    korgMessageReceived : function(msg) {
        // ["TILT","PAN","LIFT","UPPER","ELBOW","FOREARM","WRIST","GRIP","TORSO"],
        for (i in this.sliderList) {
            var name = this.sliderList[i];
            this.slider[name].setValue(msg.axes[i]);
        }
        this.last_msg = msg;
    },

    onVirtualKorgChanged : function() {
        var msg = {};
        if (this.last_msg && this.last_msg.axes) {
            msg.axes = this.last_msg.axes;
        } else {
            msg.axes = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0 ];
        }
        if (this.last_msg.buttons) {
            msg.buttons = this.last_msg.buttons;
        } else {
            msg.buttons = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];
        }

        for ( var i in this.sliderList) {
            var name = this.sliderList[i];
            msg.axes[i] = this.slider[name].value;
        }

        this.korgChanged(msg);
    },

});
