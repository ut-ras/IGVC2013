dojo.provide("cs148widgets.CodePanel");

dojo.require("cs148widgets.PythonEditor");
dojo.require("cs148widgets.PythonExecutor");

dojo.require("dijit.layout.ContentPane");
dojo.require("dijit.form.Textarea");
dojo.require("dijit.form.Button");

dojo.declare("cs148widgets.CodePanel", [ dijit.layout.ContentPane ], {
    
    // ContentPane parameters
    title: "Python",
    closable: true,
    
    postCreate: function() {
        dojo.addClass(this.domNode, "codePanel");
        
        // Create the code entry widget and executor
        this.codeExecutor = new cs148widgets.PythonExecutor();
        this.codeEntry = new cs148widgets.PythonEditor();
        this.domNode.appendChild(this.codeEntry.domNode);
        this.connect(this.codeExecutor, "onPrint", "onPrint");
        
        // Create the run and stop buttons
        this.buttons = document.createElement('div');
        this.domNode.appendChild(this.buttons);
        
        var startButton = new dijit.form.Button({ label: "Run", title: "Press CTRL+Enter to Run" });
        this.connect(startButton, "onClick", "runPython");
        this.buttons.appendChild(startButton.domNode);
        
        var stopButton = new dijit.form.Button({ label: "Stop" });
        this.connect(stopButton, "onClick", "stopPython");
        this.buttons.appendChild(stopButton.domNode);
    },
    
    runPython: function() {
        var python = this.codeEntry.editor.getValue();
        this.codeExecutor.run(python);
    },
    
    stopPython: function() {
        this.codeExecutor.stop();
    },
    
    resize: function() {
        this.inherited(arguments);
        
        // Recalculates the contents sizes
        var cb = dojo.contentBox(this.domNode);
        var buttonsHeight = this.buttons.clientHeight;
        
        var codeHeight = cb.h - buttonsHeight - 4;
        
        this.codeEntry.setHeight(codeHeight);
    },
    
    onClose: function() {
    	// If there is contents, check with the user before closing
    	return this.codeEntry.editor.getValue() ? confirm("This editor contains code. Any unsaved code will be lost.") : true;
    },
    
    // Connect to this method to receive print statements
    onPrint: function(msg) {}
    
});