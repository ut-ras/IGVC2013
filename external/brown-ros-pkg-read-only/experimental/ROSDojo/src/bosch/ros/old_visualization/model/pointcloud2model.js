/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
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

ros.visualization.PointCloud2Model = ros.visualization.Model.extend({
  init: function(gl, shader_manager, sceneviewer,node) 
  {
    this._super(gl,shader_manager);
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.POINT_CLOUD];
    //this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE];
    this.renderer = new SglMeshGLRenderer(this.prog); 
    this.pointcloud = null;
    this.mesh = null;
    this.pointstep = null;
    this.scene_viewer = sceneviewer;
    this.bparser_big = new BinaryParser(true,false);
    this.bparser_little = new BinaryParser(false,false);
    this.color = [1.0,0.0,0.0];
    this.attenuation = new Float32Array([0.01, 0.0, 0.003]);
    this.pointSize = 1.0;
      this.node = node;
    //this.topic = pointcloudtopic;
    
    //var that = this;
    //that.node.subscribe(that.topic,function(msg){ that.updateFromMessage(msg)});
  },

    // changeTopic: function(newTopic){
    // 	var that = this;
    // 	that.node.unsubscribe(that.topic);
    // 	that.node.subscribe(newTopic,function(msg){ that.updateFromMessage(msg)});
    // },

  // currently it only handles XYZ data
  updateFromMessage: function(msg) {
    console.log("received new point cloud message!");
    this.pointcloud = decodeBase64(msg.data);
    this.pointstep = msg.point_step;
    
    var num_points = msg.width * msg.height;
    var points = this.pointcloud;
    var point_step = this.pointstep;
    var length = points.length;
    console.log("point cloud length is:");
    console.log(points.length);
    var positions = [];
    var colors = [];
    var x,y,z;
    var r,g,b;
    var parser = null;
    var i,j,k,l;
    var is_rgb = false;
    var rgb;

    for(i = 0; i < msg.fields.length; i++) 
      if(msg.fields[i].name == "rgb")
        is_rgb = true;

    if(msg.is_bigendian == false)
      parser = this.bparser_little;
    else 
      parser = this.bparser_big;

    for(i=0,j=0,k=0, l=length; i < l; i+=point_step) {      
      x = this.extractFloat(i,points,parser);
      y = this.extractFloat(i+4,points,parser);
      z = this.extractFloat(i+8,points,parser);
      
      positions[j++] = x;
      positions[j++] = y;
      positions[j++] = z;


      if(is_rgb) {
        rgb = parser.toInt(points.slice(i+16,i+20));
        r = ((rgb >> 16) & 0xff);
        g = ((rgb >> 8) & 0xff);
        b = ((rgb) & 0xff);
        colors[k++] = r/255; 
        colors[k++] = g/255; 
        colors[k++] = b/255;
      }
      else {
        colors[k++] = 1.0;
        colors[k++] = 0;
        colors[k++] = 0;
      }
    }

    var mesh = new SglMeshGL(this.gl);
    mesh.addVertexAttribute("position", 3, new Float32Array(positions));
    mesh.addVertexAttribute("color", 3, new Float32Array(colors));
    mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, num_points);
    mesh.primitives = "vertices";
    this.mesh = mesh;

  },

  extractFloat : function(index,points,parser)
  {
    var ps = points.slice(index,index+4);
    var p  = parser.toFloat(ps);

    return p;
  },
   
  load: function (callback, scene_viewer)
  {
    var positions = new Float32Array([-0.27104,0.20824,0.47563,
                                       0.47547,0.20872,0.47609,
                                      -0.27143,0.21244,0.47213]);
    var colors = new Float32Array([1.0,0,0,
                                   1.0,0,0,
                                   1.0,0,0]);
    var num_points = 3;
    this.mesh = new SglMeshGL(this.gl);
    this.mesh.addVertexAttribute("position", 3, positions);
    this.mesh.addVertexAttribute("color", 3, colors);
    this.mesh.addArrayPrimitives("vertices", this.gl.POINTS, 0, num_points);
    this.mesh.primitives = "vertices";
    
//    // update bounding box 
//    this.updateBoundingBox(positions);
    
    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
    
  },

  draw: function (gl, xform)
  {
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix, u_mv : xform.modelViewMatrix, u_pointsize : this.pointSize, u_attenuation : this.attenuation};

    gl.disable(gl.BLEND);
    gl.enable(gl.VERTEX_PROGRAM_POINT_SIZE);
    sglRenderMeshGLPrimitives(this.mesh, "vertices", this.prog, null, uniforms); 
    gl.disable(gl.VERTEX_PROGRAM_POINT_SIZE);
    
  },
  
});

