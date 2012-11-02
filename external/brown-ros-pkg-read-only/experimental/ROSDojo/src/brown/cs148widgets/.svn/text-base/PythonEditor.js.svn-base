dojo.provide("cs148widgets.PythonEditor");

dojo.require("dijit._Widget");
dojo.require("cs148widgets.Utils");

dojo.declare("cs148widgets.PythonEditor", [ dijit._Widget ], {
    
    postCreate: function() {        
        // Create a CodeMirror editor instance
        this.editor = CodeMirror(this.domNode, {
            mode: "python",
            lineNumbers: true,
            matchBrackets: true,
            gutter: true,
            tabSize: 4,
            indentUnit: 4,
            extraKeys: {"Tab": "indentMore", "Shift-Tab": "indentLess"}
        });
        
        dojo.addClass(this.domNode, "pythonEditor dijitTextBox");
    },
    
    setHeight: function(height) {
        dojo.style(this.domNode, "height", height+"px");
        this.editor.refresh();
    }
});