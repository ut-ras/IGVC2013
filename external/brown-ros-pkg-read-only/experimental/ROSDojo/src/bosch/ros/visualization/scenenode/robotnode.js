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

ros.visualization.RobotNode = ros.visualization.SceneNode.extend({
  init: function(vm,urdf_xml,alpha)
  {
    this._super(vm);
    this.prog = vm.shader_manager.shaderPrograms[vm.shader_manager.ShaderTypes.PHONG];
    this.renderer = new SglMeshGLRenderer(this.prog);

    this.urdf_xml = urdf_xml;
    this.light = false;

    if(alpha == "undefined" || alpha == null)
      this.alpha = 1;
    else 
      this.alpha = alpha;

    this.name = "Robot";
  },

  urdfPoseToMatrix: function (pose)
  {
    var matrix = sglIdentityM4();
    var translation = sglTranslationM4V([pose.position.x, pose.position.y, pose.position.z]);
    var rotation = sglGetQuatRotationM4([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]);
    matrix = sglMulM4(matrix,translation);
    matrix = sglMulM4(matrix,rotation);
    return matrix;
  },


  load: function (callback, scene_viewer)
  {
    var that = this;
    var urdf_model = new ros.urdf.Model();

    function onLoadUrdfModel() {
      // load all models
      var links = urdf_model.getLinks().valSet();
      for( var l in links) {
        var link = links[l];
        if(!link.visual) continue; 
        if(!link.visual.geometry) continue;
        if(link.visual.geometry.type == link.visual.geometry.GeometryTypes.MESH) {
          var frameId = new String("/"+link.name);
          var uri = link.visual.geometry.filename.replace("package://",dojo.moduleUrl("bosch","resources/"));
          
          // ignore mesh files which are not in collada format
          if (uri.substr(-4).toLowerCase() != ".dae") {
            ros_error(uri + " is not a valid collada file!");
            continue;
          }

          // create a collada model
          var scene = { frame:frameId, url:uri, specular:[0.0, 0.0, 0.0, 0.0], eye:[0.0,0.0,4.0], alpha: that.alpha};
          var scene_node = new ros.visualization.SceneNode(this.vm);
          var collada_model = new ros.visualization.ColladaModel(that.vm.gl, that.vm.shader_manager, scene);

          scene_node.matrix = that.matrix;
          scene_node.matrix = sglMulM4(that.urdfPoseToMatrix(link.visual.origin),scene_node.matrix); 
          scene_node.setFrame(frameId);
          scene_node.scale = that.scale;
          scene_node.pickable = that.pickable;

          collada_model.light = this.light;

          // load collada file
          collada_model.load(this);

          if(that.light) {  
            collada_model.shader = that.vm.shader_manager.ShaderTypes.PHONG;
          }
          else {
            collada_model.shader = that.vm.shader_manager.ShaderTypes.TEXTURE;
          }

          collada_model.prog = that.vm.shader_manager.shaderPrograms[collada_model.shader];
          collada_model.renderer = new SglMeshGLRenderer(collada_model.prog);
          scene_node.setModel(collada_model);

          that.children.push(scene_node);
        }
      }
    }

    urdf_model.initFile(this.urdf_xml, onLoadUrdfModel);

    var async = (callback) ? (true) : (false);
    if(async) {
      callback(this);
    }
  },

  setLight : function(enable)
  {
    this.light = enable;

    if(this.light) {  
      collada_model.shader = that.vm.shader_manager.ShaderTypes.PHONG;
    }
    else {
      collada_model.shader = that.vm.shader_manager.ShaderTypes.TEXTURE;  
    }
  }

});
