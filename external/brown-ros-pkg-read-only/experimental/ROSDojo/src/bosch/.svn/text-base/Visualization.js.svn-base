dojo.provide("bosch.Visualization");
dojo.provide("bosch.TurtlebotVisualization");
dojo.provide("bosch.PR2Visualization");

dojo.require("bosch.Utils");

dojo.require("dijit._Widget");

dojo.declare("bosch.Visualization", dijit._Widget, {
	
	// Optional parameters
	tf_topic: "/tf_changes",
	fixed_frame: "/base_link",
	urdf: dojo.moduleUrl("bosch", "resources/pr2_description/pr2_urdf.xml"),
	width: 800,
	height: 600,
	
	postCreate: function() {
		dojo.addClass(this.domNode, "robot-visualization");
		
		this.canvas = document.createElement('canvas');
		this.canvas.id = this.id+"_canvas";
		this.canvas.width = ""+this.width;
		this.canvas.height=""+this.height;
		this.domNode.appendChild(this.canvas);		
	},
	
	startup: function() {
		// The canvas must be attached to the page before we can create the visualization		
		this.nodeHandle = new ros.NodeHandle();
		this.tf = new ros.tf.TransformListener(this.nodeHandle, this.tf_topic);
		this.vm = new ros.visualization.VisualizationManager(this.canvas.id);
		this.vm.initialize(this.nodeHandle, this.tf); 

		this.vm.scene_viewer.fixed_frame = this.fixed_frame;

  	this.connect(ros, "onOpen", "refreshRobot");
		this.periodicallyRefresh();

		this.addModels();
	},
	
	setHeight: function(height) {
		this.height = height;
		this.canvas.height = ""+height;
		dojo.style(this.domNode, "height", height+"px");
		window.setTimeout(dojo.hitch(this, "resize"), 0);
	},
	
	resize: function() {
		var size = dojo.contentBox(this.domNode);
//		if (size.w > 800) {
//			size.h = (size.h/size.w) * 800;
//			size.w = 800;
//		}
//		if (size.h > 500) {
//			size.w = (size.w / size.h) * 500;
//			size.h = 500;
//		}
		this.canvas.height = size.h+"";
		this.canvas.width = size.w+"";
	},
	
	periodicallyRefresh: function() {
		this.refreshRobot();
		window.setTimeout(dojo.hitch(this, "periodicallyRefresh"), 30000);
	},

	refreshRobot: function() {
		if (ros.available()) {
			ros.callService("/wviz_tf_manager/publish_all_transforms", "[]", function() {});
		}
	},
	
	// Returns a data URI of the image
	screenshot: function() {
		return this.canvas.toDataURL();		
	},
	
	// Returns a data URI of the image with background cropped out.
	// Due to implementation details, the data URI is passed back via the callback function
	generatePreview: function(callback) {
		// Create some variables with useful information
		var width = this.canvas.width;
		var height = this.canvas.height;
		
		// Take a screenshot and create an img
		var img = document.createElement('img');
		img.src = this.screenshot();
		
		// Create a canvas. We will place the image on the canvas, then extract the pixel details
		var canvas = document.createElement("canvas");
		canvas.width = width;
		canvas.height = height;
		var ctx = canvas.getContext("2d");
		
		// The image doesn't load immediately and we have to continue processing in a callback
		img.onload = dojo.hitch(this, function() {
			// Draw the image to the canvas and extract the pixels
			ctx.drawImage(img, 0, 0);			
			var px = ctx.getImageData(0, 0, width, height).data
			
			// Calculate the bounds of the image
			var bounds = bosch.Utils.getBounds(px, width, height, 0, 0, 0);
			if (bounds.width==0 || bounds.height==0) {
				return;
			}
			
			// Create a new canvas based with the calculated dimensions
			var newcanvas = document.createElement("canvas");
			newcanvas.width = bounds.width;
			newcanvas.height = bounds.height;
			var newctx = newcanvas.getContext("2d");
			
			// Draw the image, cropped, to the new canvas
			newctx.drawImage(canvas, bounds.left, bounds.top, bounds.width, bounds.height, 0, 0, bounds.width, bounds.height);
			
			// Call the callback with the new image's data uri
			callback(newcanvas.toDataURL());
		});
	},

	// dummy function for overriding
	addModels : function() {}

	
});

dojo.declare("bosch.TurtlebotVisualization", bosch.Visualization, {
	urdf: dojo.moduleUrl("bosch", "resources/turtlebot_description/turtlebot_urdf.xml")
})

dojo.declare("bosch.PR2Visualization", bosch.Visualization, {
	urdf: dojo.moduleUrl("bosch", "resources/pr2_description/pr2_urdf.xml")
})
