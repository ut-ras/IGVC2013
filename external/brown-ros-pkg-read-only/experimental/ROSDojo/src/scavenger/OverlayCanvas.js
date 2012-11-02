dojo.provide("scavenger.OverlayCanvas");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");
dojo.require("bosch.Visualization");
dojo.require("scavenger.Utils");

dojo.declare("scavenger.OverlayCanvas",[dijit._Widget, dijit._Templated], {
  
  vm : null,
  templatePath : dojo.moduleUrl("scavenger", "templates/OverlayCanvas.html"),

	// Optional parameters
	tf_topic: "/tf",
fixed_frame: "/wide_stereo_optical_frame",
camera_info : "/wide_stereo/left/camera_info",
camera_topic : "/wide_stereo/left/image_rect_color?quality=50",

  width: 640,
  height: 480,
  mjpeg_server :"pro",
	urdf: dojo.moduleUrl("bosch", "resources/pr2_description/pr2_urdf.xml"),
    
  postCreate: function() {
		this.canvas.width = ""+this.width;
		this.canvas.height=""+this.height;
		this.canvas.id = this.id+"_canvas";
      
    dojo.connect(ros, "onConnecting", this, "onConnecting");
  },

  startup: function() {
		// The canvas must be attached to the page before we can create the visualization		
		this.nodeHandle = new ros.NodeHandle();
		this.tf = new ros.tf.TransformListener(this.nodeHandle, this.tf_topic);
		this.vm = new ros.visualization.VisualizationManager(this.canvas.id);
		this.vm.initialize(this.nodeHandle, this.tf); 

		this.vm.scene_viewer.fixed_frame = this.fixed_frame;

    var camera_overlay = new ros.visualization.CameraOverlay(this.vm, this.fixed_frame,this.camera_info); 

    dojo.connect(this.vm.scene_viewer,"mouseDown",this,"mouseDown");
    dojo.connect(this.vm.scene_viewer,"mouseUp",this,"mouseUp");
    dojo.connect(this,"getWorldRay",this.vm.scene_viewer,"getWorldRay");

    this.addModels();
    this.extraSetup();
  },


  onConnecting: function(url) {
    this.imgAttach.src = "http://"+url + ":8080/stream?topic="+ this.camera_topic+"?width="+this.width+"?height="+this.height;
  },

  addModels : function() {
		this.vm.addRobotModel(this.urdf,0.5);
    this.vm.addGrid("/base_link",10.0,1.0);
              },

  extraSetup : function() {},

  // dummy functions to handle mouse events
  mouseDown : function(gl, button, x,y) {},
  mouseUp : function(gl, button, x,y) {},

  getWorldRay : function(x,y) {},

});

