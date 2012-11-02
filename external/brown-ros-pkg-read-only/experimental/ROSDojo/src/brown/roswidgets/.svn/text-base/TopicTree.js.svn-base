dojo.provide("roswidgets.TopicTree");

dojo.require("roswidgets.common.Tree");

dojo.declare("roswidgets.TopicTree", [ roswidgets.common.Tree ], {
    rootLabel: "Topics",
    postCreate: function() {
        this.inherited(arguments);
        dojo.addClass(this.domNode, "topiclist");
    },
    onPoll: function() {
        if (!this.deactivated) {
            ros.topics(dojo.hitch(this, "onDataReceived"));
        }
    }
});