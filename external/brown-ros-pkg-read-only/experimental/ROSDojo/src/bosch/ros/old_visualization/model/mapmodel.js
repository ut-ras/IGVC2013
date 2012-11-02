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

ros.visualization.MapModel = ros.visualization.Model.extend({
    init: function(gl, shader_manager,node) 
  {
    this._super(gl, shader_manager);
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.TEXTURE];
    this.renderer = new SglMeshGLRenderer(this.prog); 
    
    this.resolution = 1.0;
    this.width = 1.0;
    this.height = 1.0;
    this.textureOptions = {
        minFilter : this.gl.LINEAR,
        magFilter : this.gl.LINEAR,
        wrapS     : this.gl.CLAMP_TO_EDGE,
        wrapT     : this.gl.CLAMP_TO_EDGE,
        generateMipmap : false
      };
      this.node = node;
  },
   
  load: function (callback)
  {  
  	//Triangle vertices:
  	var triangle_vertices = new Float32Array([
  	  0.000000,                   0.000000,                    0.000000,
  	  this.resolution*this.width, 0.000000,                    0.000000,
  	  this.resolution*this.width, this.resolution*this.height, 0.000000,
  	  0.000000,                   this.resolution*this.height, 0.000000
  	]);
  
    //Vertex normals:
  	var triangle_normals = new Float32Array([
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000
  	]);
          
    //Triangle indices:
  	var triangle_indices = new Uint16Array
  	([
  		0, 2, 3,  // First triangle
  		0, 1, 2   // Second triangle
  	]);
          
    //Texture coordinates:
  	var triangle_texcoords = new Float32Array
  	([
  		0.000000, 0.000000,
  		1.000000, 0.000000,
  		1.000000, 1.000000,
  		0.000000, 1.000000
  	]);
  
  	// create a dummy texture
  	var dummyRedTexels = new Uint8Array([255, 0, 0, 0]);
  	var dummyRedTex = new SglTexture2D(this.gl, this.gl.RGBA, 1, 1, this.gl.RGBA, this.gl.UNSIGNED_BYTE, dummyRedTexels, this.textureOptions);
  	this.texture = dummyRedTex;

    this.vertices = triangle_vertices;
    this.indices = triangle_indices;
    
  	this.mesh = new SglMeshGL(this.gl);
  	this.mesh.addVertexAttribute("position", 3, triangle_vertices);
  	this.mesh.addVertexAttribute("texcoord", 2, triangle_texcoords);
    this.mesh.addVertexAttribute("normal",   3, triangle_normals);
  	this.mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, 4);
  	this.mesh.addIndexedPrimitives("triangles", this.gl.TRIANGLES, triangle_indices);

    this.updateBoundingBox(triangle_vertices);
  	
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
  },

  draw: function (gl, xform)
  {
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix };
    var samplers = { s_texture : this.texture };
    sglRenderMeshGLPrimitives(this.mesh, "triangles", this.prog, null, uniforms, samplers);
  },

  updateFromMessage: function (grid_map_msg)
  {
    ros_debug("Received a " + grid_map_msg.info.width + " X " + grid_map_msg.info.height + " map @ " + grid_map_msg.info.resolution + " m/pix");
    
    this.resolution = grid_map_msg.info.resolution;
    this.width = grid_map_msg.info.width;
    this.height = grid_map_msg.info.height;
    
    //Expand it to be RGB data
    var pixels_size = this.width * this.height;
      var array = new Array();
      var width = this.width;
      var height = this.height;

    for(var j=0;j<height;j++)
    {
      var jj = height - 1 - j;
      for(var i=0;i<width;i++)
      {
        var grid_data = grid_map_msg.data[j*width+i];
	//console.log("Grid data is:");
	//console.log(grid_data);
	
        var val;
        if(grid_data == 100)
          val = 0;
        else if(grid_data == 0)
          val = 255;
        else
          val = 127;
  
        var pidx = (jj*width + i)*4;
        array[pidx] = val;
        array[pidx+1] = val;
        array[pidx+2] = val;
        array[pidx+3] = 0;
      
      }
    }
    console.log("In Model; array is:");
    console.log(array);
    
//    var img = new ros.geometry.TextureImage(array,width,height);
    var img = new Uint8Array(array);
    //   console.log("IMG");
    //   console.log(img);
    //   console.log("WIDTH");
    //   console.log(width);
    //   console.log("HEIGHT");
    //   console.log(height);
      var texture = new SglTexture2D(this.gl, this.gl.RGBA, width, height, this.gl.RGBA, this.gl.UNSIGNED_BYTE, img, this.textureOptions);
      
    this.texture.destroy();
    //console.log("TEXTURE");
    //console.log(texture)
    this.texture = texture;

    // create a dummy texture
    //var dummyRedTexels = new Uint8Array([255, 0, 0, 0,255, 255, 0, 0,0, 255, 0, 0, 0, 0, 255, 0]);
      //var dummyGreenTexels = new Uint8Array([0, 255, 0, 0]);
    //var dummyGreenTex = new SglTexture2D(this.gl, this.gl.RGBA, 1, 1, this.gl.RGBA, this.gl.UNSIGNED_BYTE, dummyGreenTexels, this.textureOptions);
    //this.texture = dummyGreenTex;
  },
});
