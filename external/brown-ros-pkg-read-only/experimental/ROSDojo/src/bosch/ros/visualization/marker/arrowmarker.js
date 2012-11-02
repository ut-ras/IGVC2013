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

ros.visualization.Markers.ArrowMarker = ros.visualization.Markers.Marker.extend({
  init: function(vm) 
  {
    this._super(vm);
    this.model = null;
  },

setScale: function(scale)
  {
    this.scale = [scale[0] * 3, scale[1] * 3, scale[2]];
  },
 
  updateFromMessage: function(marker_msg) 
  {
    this._super(marker_msg);
    this.setFrame(marker_msg.header.frame_id);
    
    if (marker_msg.points.length > 0 && marker_msg.points.length < 2)
    {
      ros_debug("Arrow marker [" + this.getStringID() + "] only specified one point of a point to point arrow.");
      this.model = null;
      return;
    }
    
    this.model = new ros.visualization.ArrowModel(this.vm.gl,this.vm.shader_manager);  
    
    if(marker_msg.points.length == 0)
    {
      if (marker_msg.scale.x * marker_msg.scale.y * marker_msg.scale.z == 0.0)
      {
        ros_debug("Scale of 0 in one of x/y/z");
      }
      this.setPose(marker_msg.pose);
      this.setFrame(marker_msg.header.frame_id);
      this.setScale([marker_msg.scale.x,marker_msg.scale.y,marker_msg.scale.z]);

      var orient = ros.math.Vector3.NEGATIVE_UNIT_Z.getRotationTo(new ros.math.Vector3(1,0,0));
      this.setOrientation(orient);

    }
    else
    {
      var p1 = marker_msg.points[0];
      var p2 = marker_msg.points[1];
      var point1 = new ros.math.Vector3(p1.x, p1.y, p1.z);
      var point2 = new ros.math.Vector3(p2.x, p2.y, p2.z);
      var axis = new ros.math.Quaternion(marker_msg.pose.orientation.w,
                                         marker_msg.pose.orientation.x,
                                         marker_msg.pose.orientation.y,
                                         marker_msg.pose.orientation.z);
      var tp1 = axis.multiplyVector(point1);
      var tp2 = axis.multiplyVector(point2);
      
      var direction = tp2.subtract(tp1);
      var distance = direction.length();
      direction.normalise();
      var orient = ros.math.Vector3.NEGATIVE_UNIT_Z.getRotationTo( direction );
      this.setPosition(tp1);
      this.setOrientation(orient);
//      this.setScale([1.0,1.0,1.0]);
      this.setScale([marker_msg.scale.x,marker_msg.scale.y,marker_msg.scale.z]);
    }
    
    if(marker_msg.color.a == 0)
      marker_msg.color.a = 0.1; 
    this.model.setColor([marker_msg.color.r, marker_msg.color.g, marker_msg.color.b, marker_msg.color.a]);
//    this.model.setColor([marker_msg.color.r, marker_msg.color.g, marker_msg.color.b]);

    this.setCumMatrix();
    
  },
  
  load: function (callback)
  {  
    this.model.load();
//    this.matrix = sglMulM4(this.matrix, sglRotationAngleAxisM4C(-Math.PI/2.0, 0, 1.0, 0));
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
  },
  
});


   
