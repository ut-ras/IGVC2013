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

ros.visualization.LineStripModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager)
  {
    this._super(gl, shader_manager);
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE_COLOR];
    this.mesh = null;
    this.points = [];
    this.colors = [];
    this.hasPerPointColor = false;
    this.is_loaded = false;
  },

  load: function(callback)
  {
    var numPoints = this.points.length;
    this.hasPerPointColor = (this.colors.length == this.points.length);
    
    var positions = [];
    for(var i=0, j=0, l=numPoints; i < l; i++) {
      var point = this.points[i];
      positions[j++] = point.x;
      positions[j++] = point.y;
      positions[j++] = point.z;
    }
    
    var colors = [];
    if(this.hasPerPointColor) {
      for(var i=0, j=0, l=numPoints; i < l; i++) {
        var color = this.colors[i];
        colors[j++] = color.r;
        colors[j++] = color.g;
        colors[j++] = color.b;
        colors[j++] = color.a;
      }
    }
    else {
      for(var i=0, j=0, l=numPoints; i < l; i++) {
        var color = this.colors[i];
        colors[j++] = this.color.r;
        colors[j++] = this.color.g;
        colors[j++] = this.color.b;
        colors[j++] = this.color.a;
      }
    }
    
    this.mesh = new SglMeshGL(this.gl);
    this.mesh.addVertexAttribute("position", 3, new Float32Array(positions));
    this.mesh.addVertexAttribute("color", 4, new Float32Array(colors));
    this.mesh.addArrayPrimitives("vertices", this.gl.LINE_STRIP, 0, numPoints);
    //This can be POINTS, LINE_STRIP, LINE_LOOP, LINES, TRIANGLE_STRIP, TRIANGLE_FAN and TRIANGLES

    this.vertices = positions;

    this.updateBoundingBox(positions);
    
    this.is_loaded = true;
    
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
  },

  draw: function (gl, xform)
  {
    if(this.is_loaded) {
      var uniforms = { u_mvp : xform.modelViewProjectionMatrix};
      sglRenderMeshGLPrimitives(this.mesh, "vertices", this.prog, null, uniforms);  
    }
  },
});
