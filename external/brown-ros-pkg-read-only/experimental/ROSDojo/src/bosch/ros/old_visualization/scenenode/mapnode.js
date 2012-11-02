/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

ros.visualization.MapNode = ros.visualization.SceneNode.extend({
    init: function(vm, args) 
  {
      console.log("MAPNODE ARGUMENTS");
      console.log(vm);
      console.log(args);

      if(args.length == 2){
	  this.current_frame = args[0];
	  this.topic = args[1];
	  
      }
      else{
	  console.log("NO ARGUMENTS PASSED IN TO MAPNODE");
	  this.current_frame = "/odom_combined";
	  this.topic = "";
      }
      
      console.log("MAPNODE'S SUPER IS");
      console.log(vm);
      
      this._super(vm);
      this.redraw=0;
      this.model = new ros.visualization.MapModel(vm.gl, vm.shader_manager,vm.node);  
      this.model.node.subscribe(this.topic, function(msg){ this.updateFromMessage(msg)});
      this.frame_id = this.current_frame;
      this.oldTopic = "";
      // Which attributes of this widget should be accessible from PropertiesWidget?
      this.keys={"current_frame":this.current_frame, "topic":this.topic};    
  },
    
    changeTopic: function(newTopic){
    	var that = this;
    	that.model.node.unsubscribe(that.topic);
    	that.model.node.subscribe(newTopic, function(msg){ that.updateFromMessage(msg)});
	that.oldTopic = that.topic;
	that.topic = newTopic;
    },

    unsub: function(){
	var that = this;
	that.model.node.unsubscribe(that.topic);
    },

    updateFromMessage: function(map_msg) {      
	console.log("Compressed map msg:");
	console.log(map_msg);
	
	// if(map_msg.header.frame_id){
	// 	  console.log("SETTING FRAME ID TO:");
	// 	  console.log(map_msg.header.frame_id);
	//  this.setFrame(map_msg.header.frame_id);
	
	// }
	// else{
	//  	  console.log("SETTING FRAME ID TO /world:");
	//  this.setFrame("/world");
	// }
	//this.setFrame("/odom_combined");
	this.resolution = map_msg.info.resolution;
	this.width = map_msg.info.width;
	this.height = map_msg.info.height;

	console.log("RESOLUTION");
	console.log(this.resolution);

	//this.setPose(map_msg.info.origin);
	//this.setScale([this.resolution * this.width, this.resolution * this.height, 1]);
	//this.setScale([this.width/this.resolution, this.height/this.resolution, 1]);
	
	this.setScale([this.resolution * this.width, this.resolution * this.height, 1]);
	map_msg.info.origin.position = this.position;
	map_msg.info.origin.orientation = this.orientation;  
	this.setPose(map_msg.info.origin);

	//console.log("Compressed map model:");
	//console.log(this.model);
        
	var dcmp = this.decompressMap(map_msg);
	// console.log("DCMP");
	// console.log(dcmp);
	
	this.model.updateFromMessage(dcmp);
    },

  decompressMap: function(compressed_msg)
  {

    // console.log("Compressed Map Msg:");
    // console.log(compressed_msg.data);

    // Let's keep the map header and info
    var decompressed_msg_header = compressed_msg.header;
    var decompressed_msg_info = compressed_msg.info;
      var decompressed_msg_data = new Array();//compressed_msg.info.width*compressed_msg.info.height);
      
    var decompressed_msg = compressed_msg;

    // Let's clean the map data
    var buffer = new Array();

    // console.log("Compressed map lenght:");
    // console.log(compressed_msg.data.length);

    for(var i=0;i<compressed_msg.data.length-1;i=i+2){

	// How many times are we going to repeat the occupancy grid value?
	var repeat = compressed_msg.data[i];

	// Occupancy grid value
	var gridVal = compressed_msg.data[i+1];

	 // console.log("We will repeat :");
	 // console.log(gridVal);
	 // console.log(repeat);
	 // console.log("times");
	

	// Decompress
	for(var j=0;j<repeat;j=j+1){
	    decompressed_msg_data.push(gridVal);
	}
    }

    decompressed_msg.data = decompressed_msg_data;

    // console.log("Decompressed map lenght:");
    // console.log(decompressed_msg.data.length);

    // console.log("Decompressed map data");
    // console.log(decompressed_msg.data);

    // Return the message
    return decompressed_msg;
  },
  
});


   
