dojo.provide("scavenger.PointNNavCanvas");

dojo.require("scavenger.OverlayCanvas");

dojo.declare("scavenger.PointNNavCanvas",scavenger.OverlayCanvas, {
  
	// Optional parameters
	tf_topic: "/tf_throttled",
	fixed_frame: "/wide_stereo_optical_frame",
  camera_info : "/wide_stereo/left/camera_info",
  camera_topic : "/wide_stereo/left/image_rect_color?quality=50",

//	fixed_frame: "/head_mount_kinect_rgb_optical_frame",
//  camera_info : "/head_mount_kinect/rgb/camera_info",
//  camera_topic : "/head_mount_kinect/rgb/image_color?quality=50",

  width: 640,
  height: 480,
  mjpeg_server : "pro",
	urdf: dojo.moduleUrl("bosch", "resources/pr2_description/pr2_urdf.xml"),
    
  addModels : function() {
		this.vm.addRobotModel(this.urdf,0.5);
		this.vm.addInteractiveMarker("/pr2_marker_control/update",0.5);
 //   this.vm.addGrid("/base_link",10.0,1.0);
  },

  extraSetup : function() {
    //this.connect(ros, "onOpen", "periodicallyRefresh");
    //this.periodicallyRefresh();
  },

	periodicallyRefresh: function() {
		this.refreshRobot();
		window.setTimeout(dojo.hitch(this, "periodicallyRefresh"), 5000);
	},

	refreshRobot: function() {
                  console.log("here");
			ros.callService("/wviz_tf_manager/publish_all_transforms", "[]", function() {});
	},


  // dummy functions to handle mouse events
  mouseDown : function(gl, button, x,y) {},
  mouseUp : function(gl, button, x,y) {},

});

