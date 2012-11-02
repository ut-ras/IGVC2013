dojo.provide("museum.PoseSequence");

dojo.require("dojo.dnd.Source");
dojo.require("dijit._Widget");
dojo.require("dijit._TemplatedMixin");
dojo.require("dijit._WidgetsInTemplateMixin");
dojo.require("dijit.form.Button");

dojo.require("museum.PoseSequenceEntry");
dojo.require("museum.AppDetails");

dojo.declare("museum.PoseSequence", [dijit._Widget, dijit._TemplatedMixin, dijit._WidgetsInTemplateMixin], {

	// Internal variables
	templateString: dojo.cache("museum", "templates/PoseSequence.html"),
	
	// Internal variables
	poses: null,
	lastpose: null,
	colliding: false,
	
	postCreate: function() {
		dojo.addClass(this.domNode, "pose-sequence");

        this.appDetails.getAppData = dojo.hitch(this, "getSequenceData");
        this.appDetails.getSequence = dojo.hitch(this, "getSequence");
				
		// The PoseSequenceEntry widgets that represent the saved poses
		this.poses = [];
		
		// Create the drag and drop source, and make sure that when an item is dropped, the relevant method here is called
		this.source = new dojo.dnd.Source(this.table);
		this.source.copyState = function( keyPressed, self ){ 
	        return false; 
        }
		this.connect(this.source, "onDrop", "onDrop");
		dojo.connect(this.source, "onMouseDown", this.sequence, "focus");

		// When save pose is called, we need to know what the current poses is.  So, we listen to the pose topic
		ros.subscribe("/museum/pose", dojo.hitch(this, "onPoseChanged"), 50, "museum_msgs/Pose");
		ros.subscribe("/museum/sequence_status", dojo.hitch(this, "onPoseExecuting"), 50, "std_msgs/Int32");
		ros.subscribe("/museum/collision", dojo.hitch(this, "onCollisionState"), null, "std_msgs/Bool");
		
		this.connect(this.sequence, "onkeypress", "onKeyPressed");
	},
	
	setSequenceData: function(app_string) {
	    this.clear();

		// Parse the JSON
		app = JSON.parse(app_string);
		var sequence = app.sequence;
		for (var i = 0; i < sequence.length; i++) {
			// Create the pose widget
			var pose = new museum.PoseSequenceEntry({ 
				imgPreview: sequence[i].preview, 
				pose: sequence[i].pose,
				number: i+1
			});
	        this.connect(pose, "onDblClick", dojo.hitch(this, "executeSinglePose", pose));
			
			// Add the pose to the sequence
			this.tbody.appendChild(pose.domNode);
			this.poses.push(pose);
		}
		
		// This makes sure the drag and drop is set up correctly
		this.source.sync();
	},
    
	getSequenceData: function(includePreview) {
		var app = { sequence: [] };
		for (var i = 0; i < this.poses.length; i++ ){
			var pose = this.poses[i];
			var entry = { pose: pose.pose };
			if (includePreview) {
			    entry.preview = pose.imgPreview;
			}
			app.sequence.push(entry);
		}
		return JSON.stringify(app);
	},
	
	getSequence: function() {
	    sequence = []
        for (var i = 0; i < this.poses.length; i++ ){
            var pose = this.poses[i];
            sequence.push(pose.pose);
        }	    
	    return sequence;
	},
	
	clear: function() {
        // Clear anything that exists in the app
        this.tbody.innerHTML = "";
        this.source.sync();
        this.poses = [];        
	},
	
	onPoseChanged: function(pose) {
		// Callback called when the robot pose changes.  We just save the most recent pose
		this.lastpose = pose;
	},
	
	onPoseExecuting: function(posenumber) {
		// Callback called when the robot executes a pose as part of a sequence.  If posenumber is -1, means sequence is finished
		if (this.executing_sequence) {
			posenumber = posenumber.data;
			children = this.tbody.childNodes;
			if (posenumber == -1) {
				this.executing_sequence = false;
			} else if (posenumber < children.length) {
				this._select(this.tbody.childNodes[posenumber]);
			}
		}
	},
	
	onCollisionState: function(state) {
	    this.colliding = state.data;
	},
	
	saveCurrentPose: function() {
		// Get a preview image and then save the pose and image
	    if (!this.colliding && this.lastpose) {
			this.getImagePreview(dojo.hitch(this, "_addPose", this.lastpose));			
		}
	},
	
	_addPose: function(pose, preview) {
		// Create the pose widget
		var pose = new museum.PoseSequenceEntry({ 
			imgPreview: preview, 
			pose: pose,
			number: this.poses.length+1
		});
		this.connect(pose, "onDblClick", dojo.hitch(this, "executeSinglePose", pose));
		
		// Add the pose to the sequence
		this.tbody.appendChild(pose.domNode);
		this.poses.push(pose);
		
		// This makes sure the drag and drop is set up correctly
		this.source.sync();
		
		// Scroll the newly added pose into view once the image has loaded
		window.setTimeout(dojo.hitch(dojo.window, "scrollIntoView", pose.domNode), 10);
	},
	
	_duplicatePoses: function(poses) {
	    var newposes = []
	    for (var i = 0; i < poses.length; i++) {
	        var widget = poses[i];

	        // Create the pose widget
	        var pose = new museum.PoseSequenceEntry({ 
	            imgPreview: widget.imgPreview, 
	            pose: widget.pose,
	            number: this.poses.length+1
	        });
	        this.connect(pose, "onDblClick", dojo.hitch(this, "executeSinglePose", pose));
	        
	        this.tbody.appendChild(pose.domNode);
	        newposes.push(pose);	      
	        this.poses.push(pose);
	    }
	    
	    this.source.sync();
	    if (pose) {
	        window.setTimeout(dojo.hitch(dojo.window, "scrollIntoView", pose.domNode), 10);
	    }
	    
	    this._selectMultiple(newposes);
	},
	
	executeSinglePose: function(poseSequenceEntry) {
		var duration = poseSequenceEntry.pose.duration;
		poseSequenceEntry.pose.duration = 0.1;
		ros.callServiceAsync("/museum/pose_robot", JSON.stringify([poseSequenceEntry.pose]), function() {});
		poseSequenceEntry.pose.duration = duration;		
	},
	
	executeMultiplePoses: function(poseSequenceEntries) {
		// Extract the poses from the pose widgets
		var poses = [];
		for (var i = 0; i < poseSequenceEntries.length; i++) {
			poses.push(poseSequenceEntries[i].pose);
		}
		
		// Call the service to execute the poses
		ros.callServiceAsync("/museum/execute_sequence", JSON.stringify([{poses: poses}]), function() {});
	},
	
	onDrop: function() {
		/* Overridden from dojo.dnd.Source.   This method gets called when a pose is dropped. */
		// Determine the new pose order and update the numbers as appropriate
	    this.recalculatePoseOrder();
	},
	
	recalculatePoseOrder: function() {
        var poses = [];
        for (var i = 0; i < this.tbody.childNodes.length; i++) {
            var widget = dijit.byId(this.tbody.childNodes[i].getAttribute("widgetid"));
            widget.setPoseNumber(i+1);
            poses.push(widget);
        }
        this.poses = poses;
	},
	
	setHeight: function(height) {
		dojo.style(this.domNode, "height", height+"px");
		height = dojo.contentBox(this.domNode).h;
		
		var buttonsSize = dojo.marginBox(this.buttons);
		dojo.style(this.sequence, "height", (height-buttonsSize.h)+"px");
	},
	
	toggleIconSize: function() {
		if (this.small) {
			this.small = false;
			dojo.removeClass(this.table, "small");
			this.zoomButton.set("iconClass", "pose-sequence-zoomin");
		} else {
			this.small = true;
			dojo.addClass(this.table, "small");
			this.zoomButton.set("iconClass", "pose-sequence-zoomout");
		}
	},
	
	getImagePreview: function(callback) {
		/*
		 * For the pose sequence, it is nice if we can have image previews.
		 * Any widget capable of generating previews should override this method.
		 * When it is called, a preview should be generated of the current robot state,
		 * and the callback should be called with the preview as the first argument.
		 * 
		 * This is implemented with the intention of the bosch.Visualization overriding
		 * this method.  For the museum demo, that occurs in the index.html page
		 */
		callback(null);
	},
	
	executeSequence: function() {
		// Executes the entire current sequence
		this.executing_sequence = true;
		this.executeMultiplePoses(this.poses);
		
	},
	
	executeSelection: function() {
		// Executes the currently selected poses
		var selection = this.source.getSelectedNodes();
		var poses = [];
		for (var i = 0; i < selection.length; i++) {
			var widget = dijit.byId(selection[i].getAttribute("widgetid"));
			poses.push(widget);
		}
		if (poses.length > 1) {
			// Executing as a pose sequence ensures the pose isn't interrupted
			this.executeMultiplePoses(poses);
		} else if (poses.length==1) {
			this.executeSinglePose(poses[0]);
		}
		this.executing_sequence = false;
	},
	
	selectPrevious: function() {
		var selection = this.source.getSelectedNodes();
		var node = null;
		if (selection.length > 0) {
			var last_node = selection[selection.length-1];
			node = last_node.previousSibling;
		}
		if (node == null) {
			node = this.tbody.lastChild;
		}
		if (node != null) {
			this._select(node);
		}
	},
	
	selectNext: function() {
		var selection = this.source.getSelectedNodes();
		var node = null;
		if (selection.length > 0) {
			var last_node = selection[selection.length-1];
			node = last_node.nextSibling;
		}
		if (node == null) {
			node = this.tbody.firstChild;
		}
		if (node != null) {
			this._select(node);
		}
	},
	
	stopSequenceExecution: function() {
		ros.callServiceAsync("/museum/stop_execution", "[]", function() {});	
	},
	
	_select: function(node) {
		this.source.selectNone();
		this.source._addItemClass(node, "Selected");
		this.source.selection[node.getAttribute("id")] = 1;
		this.source._removeAnchor();
	},
	
	_selectMultiple: function(poses) {
        this.source.selectNone();
        for (var i = 0; i < poses.length; i++) {
            var node = poses[i].domNode;
            this.source._addItemClass(node, "Selected");
            this.source.selection[node.getAttribute("id")] = 1;
        }
        this.source._removeAnchor();	    
	},
	
	duplicateSelected: function() {
        var selection = this.source.getSelectedNodes();
        var poses = [];
        for (var i = 0; i < selection.length; i++) {
            var widget = dijit.byId(selection[i].getAttribute("widgetid"));
            poses.push(widget);
        }
        this._duplicatePoses(poses);
	},
	
	onKeyPressed: function(data) {
        if (data && data.keyCode && !this.source.isDragging) {
            if (data.keyCode==dojo.keys.DELETE) {
                this.source.deleteSelectedNodes();
                this.recalculatePoseOrder();
            } else if (data.keyCode==dojo.keys.UP_ARROW || data.keyCode==dojo.keys.LEFT_ARROW) {
                this.selectPrevious();
                this.executeSelection();
            } else if (data.keyCode==dojo.keys.DOWN_ARROW || data.keyCode==dojo.keys.RIGHT_ARROW) {
                this.selectNext();
                this.executeSelection();
            } else if (data.keyCode==dojo.keys.ENTER || data.keyCode==dojo.keys.SPACE) {
                this.executeSelection();
            }
        }
	},
	
	onSaveClicked: function() {}
	
});