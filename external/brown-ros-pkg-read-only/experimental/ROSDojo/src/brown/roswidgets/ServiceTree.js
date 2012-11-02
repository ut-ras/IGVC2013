dojo.provide("roswidgets.ServiceTree");

dojo.require("roswidgets.common.Tree");

dojo.declare("roswidgets.ServiceTree", [ roswidgets.common.Tree ], {
    rootLabel: "Services",
    postCreate: function() {
        this.inherited(arguments);
        dojo.addClass(this.domNode, "servicelist");
    },
    onPoll: function() {
        if (!this.deactivated) {
            ros.services(dojo.hitch(this, "onDataReceived"));
        }
    }
});