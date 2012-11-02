dojo.provide("museum.KorgNanokontrol");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");

dojo.require("museum.KorgSlidersControl");
dojo.require("museum.KorgButtonControl");

dojo.declare("museum.KorgNanokontrol", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {

	// Internal variables
	templateString : dojo.cache("museum", "templates/KorgNanokontrol.html"),
	sliderController : null,
	buttonController : null,

	postCreate : function() {
        ros.subscribe("/joy", dojo.hitch(this.buttonController, "korgMessageReceived"), -1, "sensor_msgs/Joy");
        ros.subscribe("/joy", dojo.hitch(this.sliderController, "korgMessageReceived"), -1, "sensor_msgs/Joy");
	},

	onVirtualKorgChanged : function(msg) {
		ros.publish("/joy", "sensor_msgs/Joy", dojo.toJson(msg));
	}

});
