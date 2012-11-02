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
 * Class that implements orbit interaction (rotating around a focal point, while always looking at that point)                              
 * @class                                                                                     
 * @augments ros.visualization.Interactor                                                                            
 */
ros.visualization.OrbitInteractor = ros.visualization.Interactor.extend(
/** @lends ros.visualization.OrbitInteractor# */
{

	/**
	 * Initialization funciton
	 */
  init: function(camera) 
  {
    this._super(camera);
    this.trackball = new SglTrackball();
    this.trackball._matrix = [-0.2, 0, 1, 0, 1, 0, 0.2, 0, 0, 1, 0, 0, 0, -0.75, 0.2, 1];
    this.interactorMatrix = this.trackball.matrix;
  },
  
  /**
   *Function that handles key strokes
   * If R is passed the view is reset to the initial position
   */

  keyDown : function(gl, keyCode, keyString) {
    if (keyString == "R") {
      this.trackball._matrix = [-0.2, 0, 1, 0, 1, 0, 0.2, 0, 0, 1, 0, 0, 0, -0.75, 0.2, 1];
      this.interactorMatrix = this.trackball.matrix;
    }
    /*    else if(keyString == "1") {
      this.trackball.reset();
      this.interactorMatrix = this.trackball.matrix;
    }
    else if(keyString == "2") {
      var x = -1;
      var ay = (0.5 / (this.height - 1 )) * 2.0 -1.0;;
      this.trackball.reset();
      this.trackball.action = SGL_TRACKBALL_ROTATE;
      this.trackball.track(this.camera.viewMatrix,x,ay,0.0);
      this.trackball.track(this.camera.viewMatrix,x,0.5,0.0);
      this.trackball.track(this.camera.viewMatrix,0.8,0,0);
      this.trackball.action = SGL_TRACKBALL_DOLLY;
      this.trackball.track(this.camera.viewMatrix,0,0,-3);

      this.interactorMatrix = this.trackball.matrix;
    }
    else if(keyString == "3") {
      var x = -1;
      var ay = (0.5 / (this.height - 1 )) * 2.0 -1.0;;
      this.trackball.reset();
      this.trackball.action = SGL_TRACKBALL_SCALE;
      this.trackball.track(this.camera.viewMatrix,0,0,0.5);

      this.trackball.action = SGL_TRACKBALL_ROTATE;
      this.trackball.track(this.camera.viewMatrix,-Math.PI/2,0,0.0);
      this.trackball.track(this.camera.viewMatrix,Math.PI/2,0,0.0);
      this.trackball.track(this.camera.viewMatrix,0,-Math.PI/2,0.0);

      this.interactorMatrix = this.trackball.matrix;
    }
    else if(keyString == "4") {
      var x = -1;
      var ay = (0.5 / (this.height - 1 )) * 2.0 -1.0;;
      this.trackball.reset();
      this.trackball.action = SGL_TRACKBALL_SCALE;
      this.trackball.track(this.camera.viewMatrix,0,0,0.5);

      this.trackball.action = SGL_TRACKBALL_ROTATE;
      this.trackball.track(this.camera.viewMatrix,-Math.PI/2,0,0.0);
      this.trackball.track(this.camera.viewMatrix,Math.PI/2,0,0.0);
      this.trackball.track(this.camera.viewMatrix,0,-Math.PI/2,0.0);

      this.interactorMatrix = this.trackball.matrix;
      }*/

  },


  /**                                                                                          
   *Function that handles mouse interactions
   * If the left buttion is down rotate
   * If the middle button is down pan 
   */
  mouseMove : function(gl, x, y, mouseButtonsDown, keysDown) {
    
    var ax1 = (x / (this.width - 1)) * 2.0 - 1.0;
    var ay1 = (y / (this.height - 1)) * 2.0 - 1.0;

    var action = SGL_TRACKBALL_NO_ACTION;
    if ((mouseButtonsDown[0] && keysDown[17])
        || mouseButtonsDown[1]) {
      action = SGL_TRACKBALL_PAN;
    } else if (mouseButtonsDown[0]) {
      action = SGL_TRACKBALL_ROTATE;
    }

    this.trackball.action = action;
    this.trackball.track(this.camera.viewMatrix, ax1, ay1, 0.0);
    this.trackball.action = SGL_TRACKBALL_NO_ACTION;
    this.interactorMatrix = this.trackball.matrix;
  },

  /**                                                                                          
   * Function that handles mouse wheel interaction 
   */
  mouseWheel : function(gl, wheelDelta, x, y, mouseButtonsDown, keysDown) {
    var action = (keysDown[16]) ? (SGL_TRACKBALL_SCALE)
        : (SGL_TRACKBALL_DOLLY);
    var factor = (action == SGL_TRACKBALL_DOLLY) ? (wheelDelta * 0.3)
        : ((wheelDelta < 0.0) ? (1.10) : (0.90));

    this.trackball.action = action;
    this.trackball.track(this.camera.viewMatrix, 0.0, 0.0, factor);
    this.trackball.action = SGL_TRACKBALL_NO_ACTION;
    this.interactorMatrix = this.trackball.matrix;
  },

  mouseDown: function(gl,mouseButtonsDown, x, y, pickedNode)
  {
  },

  mouseUp: function(gl, x, y, button, keysDown)
  {
  },

  
});
