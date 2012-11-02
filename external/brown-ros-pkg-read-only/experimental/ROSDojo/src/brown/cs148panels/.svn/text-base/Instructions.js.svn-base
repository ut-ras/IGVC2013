dojo.provide("cs148panels.Instructions");
dojo.provide("cs148panels.EnclosureEscapeInstructions");

dojo.require("dojo.cache");
dojo.require("dijit.layout.ContentPane");

dojo.require("cs148panels.Utils");

/*
 * This is a content pane that displays instructions.  Set the instructions
 * variable to either a string or a domnode 
 */
dojo.declare("cs148panels.Instructions", [ dijit.layout.ContentPane ], {

    title: "Assignment Instructions",
    instructions: null, /* string or domnode */
    
    postCreate: function() {
        // Set the class of the main dom node
        dojo.addClass(this.domNode, "content instructions");
        
        if (this.instructions) {
            this.setInstructions(this.instructions);
        }
    },
    
    setInstructions: function(instructions) {        
        // Clear anything that may currently exist
        this.domNode.innerHTML = "";
        
        // Insert the instructions
        if (typeof instructions == "string") {
            this.domNode.innerHTML = instructions;
        } else {
            this.domNode.appendChild(instructions);
        }
    }
    
});

dojo.declare("cs148panels.EnclosureEscapeInstructions", [ cs148panels.Instructions ], {
    instructions: dojo.cache("cs148panels", "instructions/enclosureescape.html")
});