/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Robert Bosch LLC.
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

ros.visualization.ViewfacingTextModel = ros.visualization.Model.extend({
  init : function(vm)
  {
    this._super(vm.gl, vm.shader_manager);
    this.prog = vm.shader_manager.shaderPrograms[vm.shader_manager.ShaderTypes.TEXTURE_TEXT];
    this.renderer = new SglMeshGLRenderer(this.prog);
    this.resolution = 1.0;
    this.width = 1.0;
    this.height = 1.0;

    this.canvas = document.getElementById('view_facing');
    if(this.canvas == null) 
    {
      this.canvas = document.createElement('canvas');
      this.canvas.setAttribute('id','view_facing');
    }
    this.name = "View Facing Text Model";
    this.textureOptions = {
        minFilter : this.gl.LINEAR,
        magFilter : this.gl.LINEAR,
        wrapS     : this.gl.CLAMP_TO_EDGE,
        wrapT     : this.gl.CLAMP_TO_EDGE,
        generateMipmap : false
      };

  },

  remove : function()
  {
  },

  load : function(callback, scene_viewer)
  {
  	//Triangle vertices:
  	var triangle_vertices = 
    [
  	  0.000000,                   0.000000,                    0.000000,
  	  1.000000,                   0.000000,                    0.000000,
  	  1.000000,                   0.400000,                    0.000000,
  	  0.000000,                   0.400000,                    0.000000
  	];
  
    //Vertex normals:
  	var triangle_normals = 
    [
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000,
  	  0.000000,                   0.000000,                    1.000000
  	];
          
    //Triangle indices:
  	var triangle_indices = 
  	[
  		0, 2, 3,  // First triangle
  		0, 1, 2   // Second triangle
  	];
          
    //Texture coordinates:
  	var triangle_texcoords = 
  	[
  		0.000000, 0.000000,
  		1.000000, 0.000000,
  		1.000000, 1.000000,
  		0.000000, 1.000000
  	];
  
  	// create a dummy texture
  	var dummyBlackTexels = new Uint8Array([0, 0, 0, 0]);
  	var dummyBlackTex = new SglTexture2D(this.gl, this.gl.RGBA, 1, 1, this.gl.RGBA, this.gl.UNSIGNED_BYTE, dummyBlackTexels, this.textureOptions);
//  	this.texture = dummyBlackTex;

    this.vertices = triangle_vertices;
    this.indices =triangle_indices;
    
  	this.mesh = new SglMeshGL(this.gl);
  	this.mesh.addVertexAttribute("position", 3, new Float32Array(triangle_vertices));
  	this.mesh.addVertexAttribute("texcoord", 2, new Float32Array(triangle_texcoords));
    this.mesh.addVertexAttribute("normal",   3, new Float32Array( triangle_normals));
  	this.mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, 4);
  	this.mesh.addIndexedPrimitives("triangles", this.gl.TRIANGLES, new Uint16Array(triangle_indices));

    this.updateBoundingBox(triangle_vertices);
  	
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
  },

  draw : function(gl, xform)
  {
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix };
    var samplers = { s_texture : this.texture };
    sglRenderMeshGLPrimitives(this.mesh, "triangles", this.prog, null, uniforms, samplers);
  },
  
  updateFromMessage : function(text_msg,scale,color)
  {
    this.text = text_msg 
    
    if(scale == 0)
      scale = 1;

    var canvas = this.canvas;
    var font_size = scale * 15;
    var r = Math.floor(color.r * 255);
    var g = Math.floor(color.g * 255);
    var b = Math.floor(color.b * 255);

    canvas.width = 1;
    canvas.height = scale * 20;

    var ctx = canvas.getContext("2d");
    ctx.font =  font_size +"px times";
//    ctx.textAlign = "center";
//    ctx.textBaseline="ideographic";
    canvas.width = (ctx.measureText(this.text).width + 10);
    canvas.height = scale * 20;
//    canvas.height = scale * 500;
    ctx = canvas.getContext("2d");
    ctx.textBaseline ="top";
//    ctx.fillStyle = "rgba(255,255,255,0)";
    ctx.font = font_size + "px times";
    ctx.fillStyle = "rgba(255,0,0,0)";
    ctx.fillRect(0,0,canvas.width,canvas.height);
    ctx.fillStyle = "rgb("+r+","+g+","+b+")";
    ctx.fillText(this.text,0,0);

    var cimg = ctx.getImageData(0,0,canvas.width,canvas.height);

    var ii = new Uint8Array(cimg.data)
    var texture = new SglTexture2D(this.gl, this.gl.RGBA, canvas.width, canvas.height, this.gl.RGBA, this.gl.UNSIGNED_BYTE, ii, this.textureOptions);
    //this.texture.destroy();
    this.texture = texture;
  },

});


