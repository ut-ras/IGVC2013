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

ros.visualization.SphereModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager, radius, latitudeBands, longitudeBands) 
  {
    this._super(gl, shader_manager);
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.FLAT];
    this.renderer = new SglMeshGLRenderer(this.prog);
    
    this.radius = 0.5;
    this.latitudeBands = 10;
    this.longitudeBands = 10;
    
    var n = arguments.length;
    
    if(n == 3) {
      this.radius = radius;
    } 
    
    if(n == 5) {
      this.radius = radius;
      this.latitudeBands = latitudeBands;
      this.longitudeBands = longitudeBands;
    } 
    
    this.name = "Sphere Model";
  },
   
  load: function (callback, scene_viewer)
  { 
    var numVertices = 0;
   
    var vertexPositionData = [];
    var normalData = [];
    var textureCoordData = [];
    for (var latNumber = 0; latNumber <= this.latitudeBands; latNumber++) {
      var theta = latNumber * Math.PI / this.latitudeBands;
      var sinTheta = Math.sin(theta);
      var cosTheta = Math.cos(theta);
   
      for (var longNumber = 0; longNumber <= this.longitudeBands; longNumber++) {
        var phi = longNumber * 2 * Math.PI / this.longitudeBands;
        var sinPhi = Math.sin(phi);
        var cosPhi = Math.cos(phi);
   
        var x = cosPhi * sinTheta;
        var y = cosTheta;
        var z = sinPhi * sinTheta;
        var u = 1- (longNumber / this.longitudeBands);
        var v = latNumber / this.latitudeBands;
   
        normalData.push(x);
        normalData.push(y);
        normalData.push(z);
        textureCoordData.push(u);
        textureCoordData.push(v);
        vertexPositionData.push(this.radius * x);
        vertexPositionData.push(this.radius * y);
        vertexPositionData.push(this.radius * z);
        numVertices = numVertices + 1;
      }
    }
   
    var indexData = [];
    for (var latNumber = 0; latNumber < this.latitudeBands; latNumber++) {
      for (var longNumber = 0; longNumber < this.longitudeBands; longNumber++) {
        var first = (latNumber * (this.longitudeBands + 1)) + longNumber;
        var second = first + this.longitudeBands + 1;
        indexData.push(first);
        indexData.push(second);
        indexData.push(first + 1);
   
        indexData.push(second);
        indexData.push(second + 1);
        indexData.push(first + 1);
      }
    }
  
    this.vertices = vertexPositionData;
    this.indices = indexData;    

  	this.mesh = new SglMeshGL(this.gl);
  	this.mesh.addVertexAttribute("position", 3, new Float32Array(vertexPositionData));
  	this.mesh.addVertexAttribute("normal",   3, new Float32Array(normalData));
  	this.mesh.addArrayPrimitives("vertices", 	  this.gl.POINTS, 0, numVertices);
  	this.mesh.addIndexedPrimitives("triangles", this.gl.TRIANGLES, new Uint16Array(indexData));
  	
  	// update bounding box 
  	this.updateBoundingBox(vertexPositionData);
  	
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
    
  },

  draw: function (gl, xform)
  {
    var color = [this.color[0] * this.highlightPass, this.color[1] * this.highlightPass , this.color[2] * this.highlightPass];
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix , u_view_normal_mat : xform.viewSpaceNormalMatrix, u_color : color};
    sglRenderMeshGLPrimitives(this.mesh, "triangles", this.prog, null, uniforms);  
  },
  
});
