dojo.provide("roswidgets.VideoTopicsTree");

dojo.require("roswidgets.common.Tree");

dojo.declare("roswidgets.VideoTopicsTree", [ roswidgets.common.Tree ], {
    rootLabel: "Video Topics",
    postCreate: function() {
        this.inherited(arguments);
        dojo.addClass(this.domNode, "videotopicslist");
    },
    onPoll: function() {
        if (!this.deactivated) {
            ros.topics(dojo.hitch(this, "onDataReceived"), "sensor_msgs/Image");
        }
    }
});