dojo.provide("museum.SendToRobot");

dojo.require("dijit._Widget");

dojo.require("dijit.form.Button");

dojo.require("museum.UserApi");
dojo.require("museum.AppApi");
dojo.require("museum.AppSelector");

dojo.declare("museum.SendToRobot", [dijit._Widget], {
    
    postCreate: function() {
        dojo.addClass(this.domNode, "museum-send-to-robot");
        this.button = new dijit.form.Button({
            label: "Send to robot",
            title : "Send to robot",
            iconClass : "send-to-robot-pr2"
        });
        this.connect(this.button, "onClick", "sendToRobot");
        this.domNode.appendChild(this.button.domNode);
    },
    
    setAppDetails: function(appDetails) {
        this.appDetails = appDetails;
        this.appDetails.displayQueueStatus = dojo.hitch(this, "displayQueueStatus");
    },
    
    sendToRobot: function() {
        if (this.appDetails) {
            this.appDetails.queueApp();
        }
    },
    
    displayQueueStatus: function(message) {
        dijit.Tooltip.show(message, this.button.domNode, ["after", "below"]);
        window.setTimeout(dojo.hitch(dijit.Tooltip, "hide", this.button.domNode), 5000);
    },
    
});
