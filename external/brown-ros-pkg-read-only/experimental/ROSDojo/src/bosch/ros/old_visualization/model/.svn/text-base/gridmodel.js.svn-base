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

ros.visualization.GridModel = ros.visualization.Model.extend({
    init: function(gl, shader_manager, size, resolution, color) 
    {
	this._super(gl, shader_manager);
	this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE];
	this.renderer = new SglMeshGLRenderer(this.prog);
	this.size = size;
	this.resolution = resolution;
	
	if(color == undefined || color == null){
	    this.setColor([0.129,0.372,0.545]);
	}
	else{    
	    this.setColor(color);
	}
	
	ros_debug("Grid color is: ");
	console.log(color);
	this.name = "Grid Model"
    },
    
    load: function (callback, scene_viewer)
    { 
	var vertexPositionData = [];
	var indexData = [];
	var size_2=this.size/2.0;
	var vertexCounter = 0;
	for(var grid_x = -size_2; grid_x <= size_2; grid_x+=this.resolution) {
	    // first vertex
	    vertexPositionData.push(grid_x);
	    vertexPositionData.push(-size_2);
	    vertexPositionData.push(0);
	    indexData.push(vertexCounter); vertexCounter++;
	    // second vertex
	    vertexPositionData.push(grid_x);
	    vertexPositionData.push(size_2);
	    vertexPositionData.push(0);
	    indexData.push(vertexCounter); vertexCounter++;
	}
	for(var grid_y = -size_2; grid_y <= size_2; grid_y+=this.resolution) {
	    // first vertex
	    vertexPositionData.push(-size_2);
	    vertexPositionData.push(grid_y);
	    vertexPositionData.push(0);
	    indexData.push(vertexCounter); vertexCounter++;
	    // second vertex
	    vertexPositionData.push(size_2);
	    vertexPositionData.push(grid_y);
	    vertexPositionData.push(0);
	    indexData.push(vertexCounter); vertexCounter++;
	}
	
	this.mesh = new SglMeshGL(this.gl);
  	this.mesh.addVertexAttribute("position", 3, new Float32Array(vertexPositionData));
  	this.mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, vertexCounter);
  	this.mesh.addIndexedPrimitives("edges", this.gl.LINES, new Uint16Array(indexData));
  	
	// update bounding box 
	this.updateBoundingBox(vertexPositionData);
	
	var async = (callback) ? (true) : (false);
	if (async) {
	    callback(this);
	}
	
    },

    draw: function (gl, xform)
    {
	var uniforms = { u_mvp : xform.modelViewProjectionMatrix , u_view_normal_mat : xform.viewSpaceNormalMatrix, u_color : this.color};
	sglRenderMeshGLPrimitives(this.mesh, "edges", this.prog, null, uniforms);
    },
    
});
