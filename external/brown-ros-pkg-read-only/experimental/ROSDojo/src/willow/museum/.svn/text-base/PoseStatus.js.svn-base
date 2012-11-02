dojo.provide("museum.PoseStatus");

dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit.Tooltip");

dojo.declare("museum.PoseStatus", [dijit._Widget, dijit._TemplatedMixin], {

	// Internal variables
	templateString: dojo.cache("museum", "templates/PoseStatus.html"),
	colliding: false,
	recordPressed: false,
	
	postCreate: function() {
        ros.subscribe("/museum/pose", dojo.hitch(this, "onPoseChanged"), 50, "museum_msgs/Pose");
        ros.subscribe("/museum/collision", dojo.hitch(this, "onCollisionStateReceived"), null, "std_msgs/Bool");	  
        ros.subscribe("/joy", dojo.hitch(this, "onKorgStateReceived"), -1, "sensor_msgs/Joy");  
	},
	
	setColliding: function(isColliding) {
	    if (isColliding) {
	        dojo.addClass(this.domNode, "collision");
	    } else {
	        dojo.removeClass(this.domNode, "collision");
	    }
	},
	
	onRecord: function() {
	    if (this.colliding) {
	        dijit.Tooltip.show("Cannot save this pose! The robot is colliding with itself!", this.domNode, ["above"]);
	        window.setTimeout(dojo.hitch(dijit.Tooltip, "hide", this.domNode), 8000);	        
	    }
	},
	
	onPoseChanged: function(pose) {
	    this.setPose(pose);
	},
	
	onKorgStateReceived: function(state) {
	    recordPressed = state.buttons[23];
	    if (recordPressed != this.recordPressed) {
	        this.recordPressed = recordPressed;
	        if (recordPressed) {
	            this.onRecord();
	        }
	    }
	},
	
	onCollisionStateReceived: function(state) {
	    if (state.data != this.colliding) {
	        this.setColliding(state.data);
	        this.colliding = state.data;	        
	    }
	},
    
    setPose: function(pose) {
        this.statusAttach.innerHTML = "";
        var fragment = document.createDocumentFragment();
        fragment.appendChild(document.createTextNode("duration("+pose.duration.toFixed(1)+"); "));
        for (var x in pose.head) {
            fragment.appendChild(document.createTextNode(x+"("+pose.head[x].toFixed(1)+"); "));
        }
        for (var x in pose.left) {
            fragment.appendChild(document.createTextNode(x+"("+pose.left[x].toFixed(1)+"); "));
        }
        for (var x in pose.right) {
            fragment.appendChild(document.createTextNode(x+"("+pose.right[x].toFixed(1)+"); "));
        }
        this.statusAttach.appendChild(fragment);
    },
	
});