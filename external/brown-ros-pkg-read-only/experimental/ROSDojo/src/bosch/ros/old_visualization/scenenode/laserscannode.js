ros.visualization.LaserScanNode = ros.visualization.SceneNode.extend({
    init: function(vm,args) 
    {
	console.log("LASERSCANNODE ARGUMENTS");
	console.log(vm);
	console.log(args);

	
	
	if(args.length == 1){
	    this.current_frame = args[0];
	    this.topic = args[1];
	}
	else{
	    console.log("NO ARGUMENTS PASSED IN TO LASERSCANNODE");
	    this.current_frame = "/odom_combined";
	    this.topic = "";
	}

	console.log("LASERSCANNODE'S SUPER IS");
	console.log(vm);

	this._super(vm);      
	this.redraw=0;

	this.model = new ros.visualization.LaserScanModel(vm.gl, vm.shader_manager,vm.scene_viewer,vm.node);

	this.model.node.subscribe(this.topic, function(msg){ this.updateFromMessage(msg)});
	this.frame_id = this.current_frame;
	this.oldTopic = "";
	// Which attributes of this widget should be accessible from PropertiesWidget?
	this.keys={"current_frame":this.current_frame, "topic":this.topic};
    },
    
    changeTopic: function(newTopic){
    	var that = this;
    	that.model.node.unsubscribe(that.oldTopic);
      console.log("changetopic");
      console.log(that.oldTopic);
    	that.model.node.subscribe(newTopic,function(msg){ that.updateFromMessage(msg)});
	that.oldTopic = that.topic;
	that.topic = newTopic;
    },

    unsub: function(){
	var that = this;
	that.model.node.unsubscribe(that.topic);
    },
    
    updateFromMessage: function(msg) 
    {
	this.setFrame(msg.header.frame_id);
	// You can do something with the message here
	// Then pass it to model's callback method
	this.model.updateFromMessage(msg);
    },
});
