dojo.provide("roswidgets.MessageEditor");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.MessageEditor", [ dijit._Widget ], {
    
    // Can provide a message example, the name of the message type, or the topic
    example: null,
    type: "",
    topic: "",
    
    // Internal variables
    count: 0,
    
    postCreate : function() { 
        dojo.addClass(this.domNode, "messageeditor");
        
        this.widgets = [];
        
        if (this.topic) {
            this.setTopic(this.topic);
        } else {
            this.clear();
        }
    },
    
    setTopic: function(topic) {
        this.clear();
        if (topic) {
            ros.topicType(topic, dojo.hitch(this, "_topicTypeCallback", this.count, topic));
        }
    },
    
    setType: function(msgType, topic) {
        this.clear();
        if (msgType) {
            console.log("Getting an example of ", msgType);
            ros.msgExample(msgType, dojo.hitch(this, "_messageExampleCallback", this.count, topic, msgType));
        }
    },
    
    setExample: function(msgExample, topic, msgType) {
        this.clear();
        // TODO:
        dojo.require("dijit.InlineEditBox");
        var pretty = this.prettyJSON(msgExample);
        var numbers = dojo.query(".string, .boolean, .null, .number", pretty);
        for (var i = 0, len = numbers.length; i < len; i++) {
            this.widgets.push(new dijit.InlineEditBox( { value: numbers[i].innerHTML} , numbers[i]));            
        }
        this.domNode.appendChild(pretty);
        
        dojo.require("dijit.form.Button");
        var button = new dijit.form.Button({ label: "Publish" });
        this.widgets.push(button);
        this.domNode.appendChild(button.domNode);
        
        this.connect(button, "onClick", function() {
            var json = pretty.textContent;
            ros.publish(topic, msgType, json);
        });
    },

    _topicTypeCallback: function(count, topic, data) {
        if (this.count==count) {
            this.setType(data, topic);
        }
    },
    
    _messageExampleCallback: function(count, topic, msgType, data) {
        if (this.count==count) {
            this.setExample(data, topic, msgType);
        }
    },
        
    clear: function() {
        this.count++;
        for (var i = 0, len = this.widgets.length; i < len; i++) {
            this.widgets[i].destroy();
        }
        this.widgets = [];
        this.domNode.innerHTML = "";
    },
        
    // Courtesy http://stackoverflow.com/questions/4810841/json-pretty-print-using-javascript
    prettyJSON: function(json) {
        json = JSON.stringify(json, undefined, 3);
        json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        var replaced = json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
            var cls = 'number';
            if (/^"/.test(match)) {
                if (/:$/.test(match)) {
                    cls = 'key';
                } else {
                    cls = 'string';
                }
            } else if (/true|false/.test(match)) {
                cls = 'boolean';
            } else if (/null/.test(match)) {
                cls = 'null';
            }
            return '<span class="' + cls + '">' + match + '</span>';
        });
        var pre = document.createElement('pre');
        dojo.addClass(pre, "prettyjson");
        pre.innerHTML=replaced;
        return pre;
    },
    
    uninitialize : function() {
        this.count++;
    }

});