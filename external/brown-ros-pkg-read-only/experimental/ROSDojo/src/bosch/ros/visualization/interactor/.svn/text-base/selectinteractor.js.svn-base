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

ros.visualization.SelectInteractor = ros.visualization.Interactor.extend({
  init: function(camera,sv) 
  {
    this._super(camera);
    this.sceneviewer = sv;
    this.trackball = new SglTrackball();
    this.interactorMatrix = this.trackball.matrix;
    
    this.pickedControl = null;

    this.lastX;
    this.lastY;

    this.mouseLock = false;
  },
   
  keyDown : function(gl, keyCode, keyString) {
  },

  // mode 1 : mouseMove, mode 2 : mouseUp
  callControlEvent : function(x, y, mouseButtonsDown, mode)
  {
    var mouse_ray = this.sceneviewer.getWorldRay(x,y) ; 
    var last_mouse_ray = this.sceneviewer.getWorldRay(this.lastX, this.lastY);
    var matrix = this.getTransformMatrix(this.pickedControl.parent.frame_id);
    
    mouse_ray.near = ros.visualization.transformPoint(matrix,mouse_ray.near);
    mouse_ray.far = ros.visualization.transformPoint(matrix,mouse_ray.far);
    last_mouse_ray.near = ros.visualization.transformPoint(matrix,last_mouse_ray.near);
    last_mouse_ray.far = ros.visualization.transformPoint(matrix,last_mouse_ray.far);

    this.pickedControl.mouseEvent(mouse_ray, last_mouse_ray, mouseButtonsDown, mode);
  },

  getTransformMatrix : function(frame_id)
  {
    var matrix = this.sceneviewer.lookupTransform(frame_id);
    
    return sglInverseM4(matrix);
  },

  mouseMove : function(gl, x, y, mouseButtonsDown, keysDown) {

    if(this.mouseLock == true)
      return;
    this.mouseLock = true;
    
    var diffX = x - this.lastX;
    var diffY = y - this.lastY;

    if(this.pickedControl == null)
      return;
    
    this.callControlEvent(x,y, mouseButtonsDown,1);
    this.lastX = x;
    this.lastY = y;

    this.mouseLock = false;
  },

  mouseUp : function(gl, x, y, button, keysDown) { 
    var pass;
    var pickInfo = this.sceneviewer.pick(gl, button, x, y);

    var mouseButton = [];

    if(button == 0)
      mouseButton[0] = true;
    else
      mouseButton[0] = false;
    
    if(this.pickedControl) {
      this.pickedControl.setHighlightPass(1.0);
      pass = this.pickedControl.parent.mouseEvent(this.pickedControl, this.pickedControl.name, pickInfo, mouseButton, ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MOUSE_UP);
      if(!pass) {
        this.callControlEvent(x,y,mouseButton,2);
      }
      this.pickedControl = null;
    }

  },

  mouseWheel : function(gl, wheelDelta, x, y, mouseButtonsDown, keysDown) {
  },

  mouseDown : function(gl, mouseButtonsDown,x,y, pickedNode) {
//    var pickedNode = this.sceneviewer.pick(gl, mouseButtonsDown, x, y);

    for(var l in pickedNode.linkarray)
    {
      var node = pickedNode.linkarray[l];

      if(node.interaction_mode > 0)
      {
        node.setHighlightPass(1.5);
        this.pickedControl = node;

        var pass = node.parent.mouseEvent(node, node.name, pickedNode, mouseButtonsDown, ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MOUSE_DOWN);

      }
    }

    this.lastX = x;
    this.lastY = y;
  },
});
