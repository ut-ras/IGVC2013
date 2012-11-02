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

ros.visualization.LaserScanModel = ros.visualization.Model.extend({
  init: function(gl, shader_manager, sceneviewer,node,laserscantopic) 
  {
    this._super(gl,shader_manager);
    this.prog = shader_manager.shaderPrograms[shader_manager.ShaderTypes.SIMPLE];
    this.renderer = new SglMeshGLRenderer(this.prog); 
    this.ranges = null;
    this.mesh = null;
    this.scene_viewer = sceneviewer;
    this.bparser_big = new BinaryParser(true,false);
    this.bparser_little = new BinaryParser(false,false);
    this.color = [1.0,0.0,0.0];
    this.attenuation = new Float32Array([0.01, 0.0, 0.003]);
    this.pointSize = 1.0;
    //this.topic = laserscantopic;
    this.node = node;
    //var that = this;
    //that.node.subscribe(that.topic,function(msg){ that.updateFromMessage(msg)});
  },

  // currently it only handles XYZ data
  updateFromMessage: function(msg) {
    //console.log("received new laserscan message!");
    this.ranges = msg.ranges;
    this.angleMin = msg.angle_min;
    this.angleIncrement = msg.angle_increment;
    var num_points = this.ranges.length;
    var points = this.ranges;
    var startAngle = this.angleMin; // Start angle of the scan in rad.
    var delta = this.angleIncrement; // Angle. Increment. in rad.
    var length = points.length;
    var positions = [];
    var colors = [];
    var x,y,z;
    var parser = null;
    var i,j,k,l;

    for(i=0,j=0,k=0, l=length; i < l; i+=1) {


	var alfa = startAngle + (i*delta); // startAngle is minus something, so keep adding
	
	x = points[i]*Math.cos(alfa);
	y = points[i]*Math.sin(alfa);
	z = 0;
	
	positions[j++] = x;
	positions[j++] = y;
	positions[j++] = z;

	
        colors[k++] = 1.0;
        colors[k++] = 0;
        colors[k++] = 0;

	// Call Model.setColor
	var my_color_arr = [colors[k-3],colors[k-2],colors[k-1]];
	this.setColor(my_color_arr);
      
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
    var uniforms = { u_mvp : xform.modelViewProjectionMatrix, u_mv : xform.modelViewMatrix, u_pointsize : this.pointSize, u_attenuation : this.attenuation, u_color : this.color};

    gl.disable(gl.BLEND);
    gl.enable(gl.VERTEX_PROGRAM_POINT_SIZE);
    sglRenderMeshGLPrimitives(this.mesh, "vertices", this.prog, null, uniforms); 
    gl.disable(gl.VERTEX_PROGRAM_POINT_SIZE);
    
  },
  
});

