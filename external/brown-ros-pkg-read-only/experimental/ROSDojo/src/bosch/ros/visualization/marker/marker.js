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

ros.visualization.Markers = ros.visualization.Markers || {};

ros.visualization.Markers.Marker = ros.visualization.SceneNode.extend({
  init: function(vm) {
    this._super(vm);

    this.message = null;
    this.expiration = new ros.Time();
  },
  
  getID: function() 
  {
    return this.message.id;
  },
  
  getStringID: function() 
  {
    return this.message.ns + "/" + this.message.id;
  },
  
  updateFromMessage: function(message) 
  {
    this.message = message;
    this.expiration = (new ros.Time).now();
    this.expiration.add(message.lifetime);
  },
  
  expired: function()
  {
    return (new ros.Time).now() >= this.expiration;
  },

  remove : function()
  {
  },

  getSize : function()
  {
    return this.scale[0];
  },

});

ros.visualization.Markers.MarkerTypes = {"ARROW" : 0, 
								     "CUBE" : 1, 
								     "SPHERE" : 2, 
								     "CYLINDER" : 3, 
						         "LINE_STRIP" : 4, 
						         "LINE_LIST" : 5, 
						         "CUBE_LIST" : 6, 
						         "SPHERE_LIST" : 7,
						         "POINTS" : 8, 
						         "TEXT_VIEW_FACING" : 9, 
						         "MESH_RESOURCE" : 10, 
						         "TRIANGLE_LIST" : 11};

ros.visualization.Markers.MarkerActions = {"ADD" : 0, 
		                           "MODIFY" : 1, 
		                           "DELETE" : 2};

ros.include('visualization/marker/arrowmarker');
ros.include('visualization/marker/cubemarker');
ros.include('visualization/marker/spheremarker');
ros.include('visualization/marker/cylindermarker');
ros.include('visualization/marker/pointsmarker');
ros.include('visualization/marker/linelistmarker');
ros.include('visualization/marker/linestripmarker');
ros.include('visualization/marker/trianglelistmarker');
ros.include('visualization/marker/viewfacingtextmarker');
ros.include('visualization/marker/meshmarker');
