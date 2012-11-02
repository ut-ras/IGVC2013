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

ros.visualization.PointCloudModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager, pointcloud) 
  {
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE];
    this.renderer = new SglMeshGLRenderer(this.prog); 
    this.pointcloud = pointcloud;
    this.mesh = null;
  },
   
  load: function (callback, scene_viewer)
  {
    var num_points = this.pointcloud.points.length;
    var positions = [];
    
    for(var i=0, j=0, l=num_points; i < l; i++) {
      var point = this.pointcloud.points[i];
      positions[j++] = point.x;
      positions[j++] = point.y;
      positions[j++] = point.z;
    }
    
    this.mesh = new SglMeshGL(this.gl);
    this.mesh.addVertexAttribute("position", 3, new Float32Array(positions));
    this.mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, num_points);
    
    // update bounding box 
    this.updateBoundingBox(positions);
    
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
    
  },

  draw: function (gl, xform)
  {
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix, u_color : this.color};
    sglRenderMeshGLPrimitives(this.mesh, "vertices", this.prog, null, uniforms);  
  },
  
});

