/*******************************************************************************
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2011, Robert Bosch LLC. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. * Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution. * Neither the name of the Robert Bosch nor the names
 * of its contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/

ros.visualization.Markers.ViewfacingTextMarker = ros.visualization.Markers.Marker.extend({
  init: function(vm)
  {
    this._super(vm);
    this.name = "View Facing Text Marker";
  },

  updateFromMessage : function(marker_msg)
  {
    this._super(marker_msg);

    this.setPose(marker_msg.pose);
    this.setFrame(marker_msg.header.frame_id);

    this.vm.scene_viewer.addListener(this);

    this.model = new ros.visualization.ViewfacingTextModel(this.vm);

    this.model.updateFromMessage(marker_msg.text, marker_msg.scale.z,marker_msg.color);
    this.changeReceived();
  },

  remove : function()
  {
    this.model.remove();
  },

  changeReceived : function()
  {
//    this.calculateMatrixFromPose();

    var pmatrix = sglIdentityM4();
    if(this.parent) {
      var pscale = sglScalingM4V(this.parent.scale);

      pmatrix = sglMulM4(pmatrix, this.parent.matrix);
      pmatrix = sglMulM4(pmatrix, pscale);
    }

    var viewmatrix = this.vm.scene_viewer.getViewMatrix();


    var vmatrix = ros.visualization.getRotationMat(viewmatrix);
    var mmatrix = ros.visualization.getRotationMat(pmatrix);
    var ori = this.orientation.copy();

    vmatrix = sglInverseM4(vmatrix);
    mmatrix = sglInverseM4(mmatrix); 

    var neg_z = ros.math.Vector3.NEGATIVE_UNIT_Z;
    var lookvector = sglMulM4V3(vmatrix,neg_z.toArray(),0.0);
    lookvector = sglMulM4V3(mmatrix, lookvector, 0.0);

    var negz_axis = this.orientation.neg_zAxis();
    var negz_quat= negz_axis.getRotationTo(new ros.math.Vector3(lookvector[0], lookvector[1], lookvector[2]));

    ori.multiplyQuat(negz_quat);

    var pos_y = ros.math.Vector3.UNIT_Y;
    var upvector = sglMulM4V3(vmatrix, pos_y.toArray(),0.0);
    upvector = sglMulM4V3(mmatrix, upvector, 0.0);

    var y_axis = ori.yAxis();
    var y_quat = y_axis.getRotationTo(new ros.math.Vector3(upvector[0], upvector[1], upvector[2]));

    ori.multiplyQuat(y_quat);
    
    this.setOrientation(ori);

    for(var c in this.children)
    {
      this.children[c].setCumMatrix();
    }
  },

  notify : function()
  {
    this.changeReceived();
  },
});
