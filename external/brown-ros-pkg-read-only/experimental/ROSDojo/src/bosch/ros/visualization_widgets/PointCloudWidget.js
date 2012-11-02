/*******************************************************************************
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2010, Robert Bosch LLC. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. * Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution. * Neither the name of the Robert Bosch nor the names
 * of its contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/

// Simple widget that adds a pointcloud node in the scene

PointCloudWidget = Class.extend({
  init: function(name) {
      this.type="PointCloudWidget";
      //this.current_frame="/openni_rgb_optical_frame";
      this.current_frame="";//"/kinect_torso_rgb_optical_frame";
      this.topic="";///kinect_torso/camera/rgb/points_filtered";
      this.sceneNodeId=-1;
      this.sceneNode=[];
      this.redraw=0;

      // Which attributes of this widget should be accessible from PropertiesWidget?
      this.keys={"current_frame":this.current_frame, "topic":this.topic};
  },

  setUpCallBacks:function(){
      var that=this;
		
  },
  setNode:function(node){
      this.node=node;
  },
  getParameterHtml:function(){
	  
	  htmlstring='';
	  htmlstring='<select  id="vsw_add_selector" size="'+ this.displayoptions.length + '">'
	  for(var i in this.displayoptions) {
	  	htmlstring= htmlstring + '<option value = "' + this.displayoptions[i][0] + '"> ' + this.displayoptions[i][0] + ' </option>';
	  }
	  htmlstring=htmlstring+"</select>"
	  
	  htmlstring=htmlstring+'<form id="vsw_add_selector_name" type="text">Display Name<input type\"text\" id=\"vsw_selected_name\"/> </form>';
	  htmlstring=htmlstring+'<center><button type="button", id="vsw_submit_type">  <center>Add</center>  </button><button type="button", id="vsw_cancel"> <center>Cancel</center></button></center>';
	  return htmlstring;
  },


 
});
