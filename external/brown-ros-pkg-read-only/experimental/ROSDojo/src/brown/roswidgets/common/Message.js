dojo.provide("roswidgets.common.Message");

dojo.require("dijit._Widget");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.common.Message", [ dijit._Widget ], {

    // User-defined parameters
    typedef: null, /* required */
    typedefs: null, /* required */
	           
    // Internal variables
	subwidgets: null,
	alive: true,
    
        
    postCreate: function() { 
        this.subwidgets = [];
        
        dojo.addClass(this.domNode, "message");
        
        if (this.typedef) {
            this.displayType(this.typedef, this.typedefs);
        }
    },
    
    displayType: function(typedef, typedefs) {
    	// Save the type and typedefs
    	this.typedef = typedef;
    	this.typedefs = typedefs;
    	
        // Ready the dom node
        dojo.removeClass(this.domNode, "loading");
        this.domNode.innerHTML = "";
        
        // Add a field called 'empty' if no fields
        if (!typedef || !typedef.fieldnames || typedef.fieldnames.length==0) {
            var div = document.createElement('div');
            dojo.addClass(div, "none");
            div.appendChild(document.createTextNode("None"));
            
            var row = document.createElement('div');
            dojo.addClass(row, "row");
            row.appendChild(div);
            this.domNode.appendChild(row);
            return;
        }
        
        // Calcualte rough width
        var w = this._calculateFieldNameWidth(typedef.fieldnames);
    	
    	// Display the fields of the type
    	for (var i = 0; i < typedef.fieldnames.length; i++) {
    	    var name = typedef.fieldnames[i];
    	    var type = typedef.fieldtypes[i];
    	    var arrayLen = typedef.fieldarraylen[i];
    	    var childtypedef = this.getTypeDef(type);
    	    this.addField(name, type, arrayLen, childtypedef, w);
    	}
    },
    
    addField: function(fieldname, fieldtype, arrayLen, /*optional*/ typedef, /*optional*/ fieldnamewidth) {        
        // Create the row
        var row = document.createElement('div');
        dojo.addClass(row, "row");
        
        // Create the name div, and add the field name
        var name = document.createElement('div');
        dojo.addClass(name, "name");
        name.appendChild(document.createTextNode(fieldname));
        if (fieldnamewidth) {
            dojo.style(name, "minWidth", fieldnamewidth+"px");
        }

        // Determine how to display the type of this field
        var displayType = fieldtype;
        if (arrayLen==0) {
            displayType = displayType + "[]";
        } else if (arrayLen>0) {
            displayType = displayType+"["+arrayLen+"]";
        }
        
        // Create the type def, and add the type
        var type = document.createElement('div');
        dojo.addClass(type, "type");
        type.appendChild(document.createTextNode(displayType));
        
        // If there is no typedef, that means this is an atomic type
        if (!typedef) dojo.addClass(type, fieldtype);
        else dojo.addClass(type, "complex");

        
        var clear = document.createElement('div');
        dojo.addClass(clear, "clear");
        
        row.appendChild(name);
        row.appendChild(type);
        row.appendChild(clear);
        
        this.domNode.appendChild(row);
        
        // If there's a typedef for this type, then add a child widget
        if (typedef) {
            this.addChildType(fieldname, typedef, fieldnamewidth);
        }
    },
    
    addChildType: function(fieldname, typedef, /*optional*/ fieldnamewidth) {
        // Create the row for the child type
        var row = document.createElement('div');
        dojo.addClass(row, "row");
        
        // Create the name div but make it blank (helps alignment) 
        var emptyname = document.createElement('div');
        emptyname.appendChild(document.createTextNode(fieldname));
        dojo.addClass(emptyname, "name blank");
        if (fieldnamewidth) {
            dojo.style(emptyname, "minWidth", fieldnamewidth+"px");
        }
        
        // Create the child widget
        var childWidget = new roswidgets.common.Message({ typedef: typedef, typedefs: this.typedefs });
        dojo.addClass(childWidget.domNode, "type submessage");
        
        var clear = document.createElement('div');
        dojo.addClass(clear, "clear");
        
        row.appendChild(emptyname);
        row.appendChild(childWidget.domNode);
        row.appendChild(clear);
        
        this.domNode.appendChild(row);
    },
    
    getTypeDef: function(type) {
        for (var i = 0; this.typedefs && i < this.typedefs.length; i++) {
            if (this.typedefs[i].type==type) {
                return this.typedefs[i];
            }
        }
        return null;
    },
    
    _calculateFieldNameWidth: function(fields) {
    	var max = 0;
    	for (var i = 0; i < fields.length; i++) {
    		var l = fields[i].length;
    		if (l > max) max = l;
    	}
    	var charWidth = 7;
    	return max * charWidth;
    },
    
    uninitialize: function() {
        this.alive = false;
        for (var i = 0; i < this.subwidgets.length; i++) {
            this.subwidgets[i].destroy();
        }
    }
    
});