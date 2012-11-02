dojo.provide("museum.PR2Visualization");

dojo.require("bosch.Visualization");

dojo.declare("museum.PR2Visualization", bosch.Visualization, {
	
	tf_topic: "/tf_changes",
	urdf: dojo.moduleUrl("bosch", "resources/pr2_description/pr2_urdf.xml"),

	addModels : function() {
		this.vm.addGrid('/base_link', 10.0, 1.0);
		this.vm.addRobotModel(this.urdf);
//		this.vm.addInteractiveMarker('/pr2_marker_control/update');
	},
		
});

