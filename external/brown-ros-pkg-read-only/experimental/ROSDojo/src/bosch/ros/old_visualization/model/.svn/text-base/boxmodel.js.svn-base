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

ros.visualization.BoxModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager) 
  {
    this._super(gl, shader_manager);
    this.prog_flat = shader_manager.shaderPrograms[shader_manager.ShaderTypes.FLAT];
    this.prog_simple = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE];

    
    this.radius = null;
    this.minExtent = null;
    this.maxExtend = null;
    
    var n = arguments.length;
    switch (n) {
      case 2:
        this.minExtent = [-0.5, -0.5, -0.5];
        this.maxExtend = [ 0.5,  0.5,  0.5];
      break;
      case 3:
        this.minExtent = [-arguments[2], -arguments[2], -arguments[2]];
        this.maxExtend = [ arguments[2],  arguments[2],  arguments[2]];
      break;
      case 4:
        this.minExtent = arguments[2];
        this.maxExtend = arguments[3];
      break;
      default:
        this.minExtent = [-0.5, -0.5, -0.5];
        this.maxExtend = [ 0.5,  0.5,  0.5];
      break;
    }

    this.name = "Box Model";
    
  },
   
  load: function (callback, scene_viewer)
  { 
    var minx = this.minExtent[0];
    var miny = this.minExtent[1];
    var minz = this.minExtent[2];
    var maxx = this.maxExtend[0];
    var maxy = this.maxExtend[1];
    var maxz = this.maxExtend[2];
    
    
    var vertices = [
      // Front face
       minx, miny, maxz,
       maxx, miny, maxz,
       maxx, maxy, maxz,
       minx, maxy, maxz,

      // Back face
       minx, miny, minz,
       minx, maxy, minz,
       maxx, maxy, minz,
       maxx, miny, minz,

      // Top face
       minx, maxy, minz,
       minx, maxy, maxz,
       maxx, maxy, maxz,
       maxx, maxy, minz,

      // Bottom face
       minx, miny, minz,
       maxx, miny, minz,
       maxx, miny, maxz,
       minx, miny, maxz,

      // Right face
       maxx, miny, minz,
       maxx, maxy, minz,
       maxx, maxy, maxz,
       maxx, miny, maxz,

      // Left face
       minx, miny, minz,
       minx, miny, maxz,
       minx, maxy, maxz,
       minx, maxy, minz,
    ];
   
    var vertexNormals = [
     // Front face
      0.0,  0.0,  1.0,
      0.0,  0.0,  1.0,
      0.0,  0.0,  1.0,
      0.0,  0.0,  1.0,

     // Back face
      0.0,  0.0, -1.0,
      0.0,  0.0, -1.0,
      0.0,  0.0, -1.0,
      0.0,  0.0, -1.0,

     // Top face
      0.0,  1.0,  0.0,
      0.0,  1.0,  0.0,
      0.0,  1.0,  0.0,
      0.0,  1.0,  0.0,

     // Bottom face
      0.0, -1.0,  0.0,
      0.0, -1.0,  0.0,
      0.0, -1.0,  0.0,
      0.0, -1.0,  0.0,

     // Right face
      1.0,  0.0,  0.0,
      1.0,  0.0,  0.0,
      1.0,  0.0,  0.0,
      1.0,  0.0,  0.0,

     // Left face
     -1.0,  0.0,  0.0,
     -1.0,  0.0,  0.0,
     -1.0,  0.0,  0.0,
     -1.0,  0.0,  0.0,
    ];

    var cubeTriangleIndices = [
       0, 1, 2,      0, 2, 3,    // Front face
       4, 5, 6,      4, 6, 7,    // Back face
       8, 9, 10,     8, 10, 11,  // Top face
       12, 13, 14,   12, 14, 15, // Bottom face
       16, 17, 18,   16, 18, 19, // Right face
       20, 21, 22,   20, 22, 23  // Left face
    ];
    
    var cubeEdgeIndices = [
      0, 1,  1, 2,  2, 3,  3, 0,  0, 2, // Front face
      4, 5,  5, 6,  6, 7,  7, 4,  4, 6, // Back face
      8, 9,  9,10, 10,11, 11, 8,  8,10, // Top face
     12,13, 13,14, 14,15, 15,12, 12,14, // Bottom face
     16,17, 17,18, 18,19, 19,16, 16,18, // Right face
     20,21, 21,22, 22,23, 23,20, 20,22, // Left face
    ];

    this.vertices = vertices;
    this.indices = cubeTriangleIndices;
    
  	this.mesh = new SglMeshGL(this.gl);
  	this.mesh.addVertexAttribute("position",    3, new Float32Array(vertices));
  	this.mesh.addVertexAttribute("normal",      3, new Float32Array(vertexNormals));
  	this.mesh.addArrayPrimitives("vertices", 	  this.gl.POINTS, 0, 24);
  	this.mesh.addIndexedPrimitives("triangles", this.gl.TRIANGLES, new Uint16Array(cubeTriangleIndices));
  	this.mesh.addIndexedPrimitives("edges",     this.gl.LINES, new Uint16Array(cubeEdgeIndices));

    // update bounding box 
    this.updateBoundingBox(vertices);
  	
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
    
  },

  draw: function (gl, xform)
  {
    var color = [this.color[0] * this.highlightPass, this.color[1] * this.highlightPass , this.color[2] * this.highlightPass];
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix , u_view_normal_mat : xform.viewSpaceNormalMatrix, u_color : color};
    
    var prog;
    if(this.light) {
      prog = this.prog_flat; 
    }
    else {
      prog = this.prog_simple;  
    }
    
    sglRenderMeshGLPrimitives(this.mesh, this.primitives, prog, null, uniforms);    
  },
  
  
});
