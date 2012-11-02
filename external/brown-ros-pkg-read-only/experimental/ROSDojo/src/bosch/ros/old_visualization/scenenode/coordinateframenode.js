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

ros.visualization.CoordinateFrameNode = ros.visualization.SceneNode.extend({
  init: function(vm) 
  {
    this._super(vm);

    this.children =[];
   
    for(var i =0 ; i < 3; i ++) {
      var child = new ros.visualization.SceneNode(vm);
      child.setModel(new ros.visualization.ArrowModel(vm.gl, vm.shader_manager));
      child.setParent(this);
      this.children.push(child);
    }
  },
   
  load: function (callback, scene_viewer)
  { 
    this.children[0].load();
    this.children[0].model.setColor([1.0,0.0,0.0,1.0]);
//    this.children[0].matrix = sglMulM4(this.children[0].matrix, sglRotationAngleAxisM4C(-Math.PI/2.0, 0, 1.0, 0));
    
    this.children[1].load();
    this.children[1].model.setColor([0.0,1.0,0.0,1.0]);
//    this.children[1].matrix = sglMulM4(this.children[1].matrix, sglRotationAngleAxisM4C(Math.PI/2.0, 1.0, 0, 0));
    
    this.children[2].load();
    this.children[2].model.setColor([0.0,0.0,1.0,1.0]);
//    this.children[2].matrix = sglMulM4(this.children[2].matrix, sglRotationAngleAxisM4C(Math.PI, 1.0, 0, 0));

    var async = (callback) ? (true) : (false);
    if (async) {
      callback(this);
    }
  },

  setCumMatrix : function()
  {
    if(this.parent)
    {
      var matrix = sglIdentityM4();      
      var translation = sglTranslationM4V([this.position.x, this.position.y, this.position.z]); 
      var rotation = sglGetQuatRotationM4([this.orientation.x, this.orientation.y, this.orientation.z, this.orientation.w]);  
      var scale  = sglScalingM4V(this.scale);
            
      matrix = sglMulM4(matrix, translation);
      matrix = sglMulM4(matrix, rotation);
      matrix = sglMulM4(matrix, scale);

      var pmatrix = sglIdentityM4();
      var pscale  = sglScalingM4V(this.parent.scale);
            
      pmatrix = sglMulM4(pmatrix, this.parent.matrix);
      pmatrix = sglMulM4(pmatrix, pscale);
      
      this.matrix = sglMulM4(pmatrix, matrix);            
    }   
    else {
      this.calculateMatrixFromPose();
    }   

    for(var c in this.children)
    {
      this.children[c].setCumMatrix();
    }
    this.children[0].matrix = sglMulM4(this.children[0].matrix, sglRotationAngleAxisM4C(-Math.PI/2.0, 0, 1.0, 0));
    this.children[1].matrix = sglMulM4(this.children[1].matrix, sglRotationAngleAxisM4C(Math.PI/2.0, 1.0, 0, 0));
    this.children[2].matrix = sglMulM4(this.children[2].matrix, sglRotationAngleAxisM4C(Math.PI, 1.0, 0, 0));
  },


});
