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

ros.visualization.PuppetInteractor = ros.visualization.Interactor.extend({
  init: function(camera,sv) 
  {
    this._super(camera);
    this.sceneviewer = sv;
    this.trackball = new SglTrackball();
    this.interactorMatrix = this.trackball.matrix;
    
    this.pickedNode = null;
    this.oldX = null;
    this.oldY = null;
    this.oldPose = null;
    this.newPose = null;
    this.movingX = null;
    this.movingY = null;
    this.kinematicSrv = null;
    this.releaseSrv = null;

    this.TORSO_LIFT_LINK = '/torso_lift_link';
    this.R_WRIST_ROLL_LINK = '/r_wrist_roll_link';
    this.L_WRIST_ROLL_LINK = '/l_wrist_roll_link';
  },
   
  keyDown : function(gl, keyCode, keyString) {
    if (keyString == "U")
    {
      this.sceneviewer.unSelectAll();
      this.pickedNode = null;
    }
    if (keyString == "B")
    {
      var data = 2;
      this.releaseSrv.call(ros.json([data]), function(msg) {            
        log('reset pose');                                      
          });                                                           
    }
  },

  mouseMove : function(gl, x, y, mouseButtonsDown, keysDown) {
    var diffX = (x - this.oldX)/200;
    var diffY = (y - this.oldY)/200;

    if(Math.abs(this.movingX - x) < 3 && Math.abs(this.movingY - y) < 3)
      return;

    this.movingX = x;
    this.movingX = y;

    if(mouseButtonsDown[2] == true || mouseButtonsDown[0] == false || this.oldPose == null)
      return;
    
    this.requestKinematicService(diffX,diffY, mouseButtonsDown,keysDown) 
  },

  mouseUp : function(gl, x, y, button, keysDown) { 
    if(this.oldPose == null)
      return;
    var diffX = (x - this.oldX) / 200;
    var diffY = (y - this.oldY) / 200;
    this.requestKinematicService(diffX,diffY,mouseButtonsDown,keysDown);
  },

  mouseWheel : function(gl, wheelDelta, x, y, mouseButtonsDown, keysDown) {
  },

  mouseDown : function(gl, mouseButtonsDown,x,y, pickedNode) {
//    this.pickedNode = this.sceneviewer.pick(gl, mouseButtonsDown, x, y);

    this.pickedNode = pickedNode;
    this.oldX = x;
    this.oldY = y;

    if(this.pickedNode.valid) {
      // pruning unnecessary links from linkarray
      this.pruneLinks();
//      var tmp = this.sceneviewer.tf.lookupTransform(this.TORSO_LIFT_LINK,this.pickedNode.linkarray[0]);
      var tmp = this.sceneviewer.tf.lookupTransform(this.pickedNode.linkarray[0],this.TORSO_LIFT_LINK);
      if(tmp == null) {
        this.oldPose == null;
        return;
      }
      
      this.oldPose = new ros.tf.Pose();
      this.oldPose.point.x = tmp.translation.x;
      this.oldPose.point.y = tmp.translation.y;
      this.oldPose.point.z = tmp.translation.z;
      this.oldPose.orientation.x = tmp.rotation.x;
      this.oldPose.orientation.y = tmp.rotation.y;
      this.oldPose.orientation.z = tmp.rotation.z;
      this.oldPose.orientation.w = tmp.rotation.w;
    }
  },

  pruneLinks : function()
  {
    /*
    var linklist = this.pickedNode.linkarray;
    var torso_index = 3; 

    var wrist_roll_index = linklist.length-2;
    
    for(l in linklist)
    {
      if(linklist[l] == this.L_WRIST_ROLL_LINK || linklist[l] == this.R_WRIST_ROLL_LINK)
        wrist_roll_index = parseInt(l);
    }

    wrist_roll_index = wrist_roll_index + 1;
    this.pickedNode.linkarray = this.pickedNode.linkarray.slice(torso_index,wrist_roll_index);
    this.pickedNode.linkarray.reverse();
//    console.log(this.pickedNode.linkarray);

*/
    if(this.pickedNode.linkarray[0][1] == "r") {
      this.pickedNode.linkarray = new Array();
      this.pickedNode.linkarray.push(this.R_WRIST_ROLL_LINK);
      this.pickedNode.linkarray.push(this.TORSO_LIFT_LINK);
    }
    else{
      this.pickedNode.linkarray = new Array();
      this.pickedNode.linkarray.push(this.L_WRIST_ROLL_LINK);
      this.pickedNode.linkarray.push(this.TORSO_LIFT_LINK);
    }
  },

  requestKinematicService : function(diffX,diffY,mouseButtonsDown,keysDown) 
  {
    var newPose = new ros.tf.Pose();

    newPose.point.x = this.oldPose.point.x;
    newPose.point.y = this.oldPose.point.y;
    newPose.point.z = this.oldPose.point.z;
    newPose.orientation.x = this.oldPose.orientation.x;
    newPose.orientation.y = this.oldPose.orientation.y;
    newPose.orientation.z = this.oldPose.orientation.z;
    newPose.orientation.w = this.oldPose.orientation.w;

    // if both left and wheel are clicked, move z coordinate by diffY
    if(mouseButtonsDown[0] && keysDown[17])
    {
      newPose.point.z = newPose.point.z - (-diffY); 
      newPose.point.x = newPose.point.x - (-diffX); 
    }
    else if(mouseButtonsDown[0])  {     // if left click only, move x and y coordinate
      newPose.point.x = newPose.point.x - (-diffX); 
      newPose.point.y = newPose.point.y - (-diffY); 
    }

    this.newPose = newPose;
    this.callKinematicService();
  },
  
  callKinematicService : function()
  {
    var target_link = this.pickedNode.linkarray;
    var pose = this.newPose;
    pose.position = pose.point;
    pose.point = null;

    var that = this;
    this.kinematicSrv.call(ros.json([target_link, pose]), function(msg)
    {
      if(!msg || msg.error_code != 1)
      {
        if(msg)
          log("Kinematic Failed error_code = " + msg.error_code);
        return;
      }
    });
  },

  reset: function()
  {
    this.oldX = null;
    this.oldY = null;
    this.oldPose = null;
    this.newPose = null;
    this.pickedNode = null;
    this.sceneviewer.unSelectAll();
  },
});
