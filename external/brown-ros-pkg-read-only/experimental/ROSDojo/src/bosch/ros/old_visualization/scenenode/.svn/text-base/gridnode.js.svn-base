ros.visualization.GridNode = ros.visualization.SceneNode.extend({
    init: function(vm,args) 
  {
      console.log("GRIDNODE ARGUMENTS");
      console.log(vm);
      console.log(args);

      if(args.length == 5){
	  this.current_frame = args[0];
	  this.size = args[1];
	  this.resolution = args[2];
	  this.color = args[3];
      }
      else{
	  console.log("NO ARGUMENTS PASSED IN TO GRIDNODE");
	  this.current_frame = "/odom_combined";
	  this.size = 10;
	  this.resolution = Math.random();
	  this.color = [Math.random(), Math.random(), Math.random()];
      }
      console.log("GRIDNODE'S SUPER IS");
      console.log(vm);

      this._super(vm);      
      this.redraw=0;
      this.model = new ros.visualization.GridModel(vm.gl, vm.shader_manager, this.size, this.resolution, this.color);
      this.frame_id = this.current_frame;

      this.keys={"current_frame":this.current_frame, "size":this.size, "resolution":this.resolution,"color":this.color};
  },
});