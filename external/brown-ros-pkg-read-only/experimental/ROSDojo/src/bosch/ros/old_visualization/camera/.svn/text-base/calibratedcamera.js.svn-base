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

/**
 * Creates and handles a calibrated camera for 3d visualization 
 * @class
 * @augments ros.visualization.Camera
 */

ros.visualization.CalibratedCamera = ros.visualization.Camera.extend({
  init: function(vm) 
  {
    this._super();
    this.vm = vm;
    this.eye = new ros.math.Vector3(0.0,0.0,0.0);;
    this.target = new ros.math.Vector3(0.0, 0.0, 1.0);
    this.up = new ros.math.Vector3(0.0, -1.0, 0.0);

    this.setViewMatrix(this.eye,this.target,this.up);
    this.projectionMatrix = sglPerspectiveM4(this.fovYRad, this.aspectRatio, this.zNear, this.zFar);
  },

  /**
   * Sets the view matrix
   * 
   * 
   * @param eye  the position of the actual camera
   * @param target what the camera is viewing, the target point that the camera is looking at  
   * @param up the tilt of the camera
   */
  setViewMatrix : function(eye,target,up) {
    this.viewMatrix = sglLookAtM4C(   eye.x,      eye.y,    eye.z, // eye
                                   target.x,   target.y, target.z, // target
                                       up.x,       up.y,     up.z); // up vector
  },

  
  resize : function(gl, width, height) {
    this.width = width;
    this.height = height;
    this.aspectRatio = this.width / this.height;
    this.projectionMatrix = sglPerspectiveM4(this.fovYRad, this.aspectRatio, this.zNear, this.zFar);
  },

  /**
   * Update's camera parameters based upon input
   * 
   * 
   * @param camera_info 
   */
  
  update : function(camera_info) {
    
    var img_width = camera_info.width;
    var img_height = camera_info.height;
    var orientation = new ros.math.Quaternion();
//    var transform = this.vm.scene_viewer.getFixedFrameTransform('/'+camera_info.header.frame_id);
//    transform = ros.visualization.getRotationMat(transform);
    
    //align camera with robot camera coordinate frame (z forward and x right)
    var fx = camera_info.P[0];
    var fy = camera_info.P[5];

    var img_aspect = (img_width / fx)  / (img_height/ fy);
    var win_aspect = this.aspectRatio;

    // Add the camera's translation relative to the left camera (from P[3]);
    var tx = -1.0 * (camera_info.P[3] / fx);
//    var right = ros.math.Vector3.UNIT_X;
//    var new_right = sglMulM4V3(transform,right.toArray(),0.0);

    var ty = -1.0 * (camera_info.P[7] / fy);
//    var down = ros.math.Vector3.UNIT_Y;
//    var new_down = sglMulM4V3(transform,down.toArray(),0.0);

//    var eye = this.eye.copy();

//    eye.x = eye.x + new_right[0] * tx + new_down[0] * ty;
//    eye.y = eye.y + new_right[1] * tx + new_down[1] * ty;
//    eye.z = eye.z + new_right[2] * tx + new_down[2] * ty;
//    this.eye = eye;

    // calculate the view matrix
//    this.setViewMatrix(this.eye, this.target, this.up);


    // calculate the projection matrix
    var cx = camera_info.P[2];
    var cy = camera_info.P[6];

    var far = this.zFar;
    var near = this.zNear;

    var m = sglUndefM4();
    
    m[ 0] =               2.0*fx/img_width;
    m[ 1] =                        0.0;
    m[ 2] =                        0.0;
    m[ 3] =                        0.0;

    m[ 4] =                        0.0;
    m[ 5] =              2.0*fy/img_height;
    m[ 6] =                        0.0;
    m[ 7] =                        0.0;

    m[ 8] =       2.0*(0.5 - cx/img_width);
    m[ 9] =      2.0*(cy/img_height - 0.5);
    m[10] =   -(far+near) / (far-near);
    m[11] =                       -1.0;

    m[12] =                        0.0;
    m[13] =                        0.0;
    m[14] = -2.0*far*near / (far-near);
    m[15] =                        0.0;
    
    this.projectionMatrix = m;    
  },
  
});
