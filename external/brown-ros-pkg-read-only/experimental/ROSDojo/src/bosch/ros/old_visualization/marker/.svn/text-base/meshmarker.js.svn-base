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

ros.visualization.Markers.MeshMarker = ros.visualization.Markers.Marker.extend({

  init : function(vm)
  {
    this._super(vm);
    this.model = null;
    this.light = false;
    this.lifetime = new ros.Time();
    this.mesh_use_embedded_materials = false;
    this.mesh_resource = "";
  },

  updateFromMessage : function(msg)
  {

    this._super(msg);
    this.setFrame(msg.header.frame_id);
    this.mesh_resource = msg.mesh_resource;

    var uri = this.mesh_resource.replace("package://","resources/");
    
    if (uri.substr(-4).toLowerCase() != ".dae") {
      ros_error("MeshMarker : " + uri + " is not a valid collada file!");
      return;
    }

    var scene = { url:uri, specular:[0.0, 0.0, 0.0, 0.0], light:this.light, eye:[0.0,0.0,4.0], alpha:1.0};

    this.model = new ros.visualization.ColladaModel(this.vm.gl, this.vm.shader_manager, scene);
    this.setPose(msg.pose);
    this.setScale([msg.scale.x, msg.scale.y, msg.scale.z]);
  },
});
