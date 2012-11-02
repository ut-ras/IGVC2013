dojo.provide("blocknlp.CommandEntry");

dojo.require("blocknlp.SpeechEntry");
dojo.require("dijit.form.Button");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("rosdojo.rosdojo");

dojo.declare("blocknlp.CommandEntry", [ dijit._Widget, dijit._Templated ], {
    
    commandTopic: "/blocknlp/nl_input",
    commandType: "std_msgs/String",

    templateString : dojo.cache("blocknlp", "templates/CommandEntry.html"),
        
    postCreate : function() {
        this.textarea = new blocknlp.SpeechEntry({ "spellcheck": false, "autocomplete": "off", "value": "Pick up the blue block." }, this.textareaAttach);
        this.button = new dijit.form.Button({ "label": "Execute" }, this.buttonAttach);
        
        this.textarea.setDisabled(true);
        this.button.setDisabled(true);
        
        this.connect(ros, "onOpen", "onOpen");
        this.connect(ros, "onClose", "onClose");
        
        this.connect(this.button, "onClick", "sendCommand");
    },
    
    displaySpeechMessage: function() {
        console.log("Checking speech!");
        if (document.createElement("input").webkitSpeech === undefined) {
            console.log("Speech not enabled");
            var message = "Your browser doesn't support HTML 5 speech recognition :(<br><br>";
            message += "You can still manually type commands, or if you want to try <br>";
            message += "speech recognition, start Chrome using <b>--enable-speech-input</b>.";
            dijit.showTooltip(message, this.button.domNode, [ "after", "above", "below", "before" ], false, "");            
        }  else {
            console.log("Speech enabled");
        }
    },
    
    onOpen: function() {
        this.textarea.setDisabled(false);
        this.button.setDisabled(false);
    },
    
    onClose: function() {
        this.textarea.setDisabled(true);
        this.button.setDisabled(true);
    },
    
    sendCommand: function() {
        ros.publish(this.commandTopic, this.commandType, dojo.toJson({ "data": this.textarea.value }));
    }

});