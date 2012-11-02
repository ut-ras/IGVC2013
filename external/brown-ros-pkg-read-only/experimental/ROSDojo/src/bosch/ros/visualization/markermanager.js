/*******************************************************************************
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2010, Robert Bosch LLC. All rights reserved.
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

ros.visualization.MarkerManager = Class.extend({
  init: function(vm) {
    this.vm = vm;
    this.markerNodes = new ros.Map();
  },
	
  subscribeMarker: function(marker_topic) {
    // subscribe to marker topic
    var that = this;
    this.vm.node.subscribe(marker_topic, function(msg){that.receiveMarkerMessage(msg);});
  },
  
  subscribeVisualizationScene: function(scene_topic) {
    // subscribe to scene topic
    var that = this;
    this.vm.node.subscribe(scene_topic, function(msg){that.receiveSceneMessage(msg);});
  },
  
  receiveSceneMessage: function(scene_msg) {
    // process scene
    for ( var c in scene_msg.markers) {
      var marker = scene_msg.markers[c];
      this.receiveMarkerMessage(marker);
    }
  },
  
  receiveMarkerMessage: function(marker_msg) {
	  var marker_id = this.getMarkerStringID(marker_msg);
	  var marker_action = marker_msg.action;
	  
//	  ros_debug("Received marker with id " + marker_id + " of type " + marker_msg.type);
	  
    var node_id = this.markerNodes.find(marker_id);
    if(node_id != null) {
      // remove existing marker from scene viewer
      this.vm.scene_viewer.removeNode(node_id);
      // remove existing marker from map
      this.markerNodes.remove(marker_id);
    }

    if (marker_action == ros.visualization.Markers.MarkerActions.DELETE) {
      // do nothing and 
      return true;
    }
    else if(marker_action == ros.visualization.Markers.MarkerActions.ADD || 
            marker_action == ros.visualization.Markers.MarkerActions.MODIFY) {
      //ros_debug("Added marker of type " + marker_msg.type);
		  var marker_type = marker_msg.type;
		  var marker = null;
	    // add a new marker
		  if (marker_type == ros.visualization.Markers.MarkerTypes.ARROW)
		    marker = new ros.visualization.Markers.ArrowMarker(this.vm);	
		  else if (marker_type == ros.visualization.Markers.MarkerTypes.CUBE)
        marker = new ros.visualization.Markers.CubeMarker(this.vm);  
		  else if (marker_type == ros.visualization.Markers.MarkerTypes.SPHERE)
        marker = new ros.visualization.Markers.SphereMarker(this.vm); 
		  else if (marker_type == ros.visualization.Markers.MarkerTypes.POINTS)
		    marker = new ros.visualization.Markers.PointsMarker(this.vm); 
		  else if (marker_type == ros.visualization.Markers.MarkerTypes.LINE_LIST)
        marker = new ros.visualization.Markers.LineListMarker(this.vm); 
		  else if (marker_type == ros.visualization.Markers.MarkerTypes.LINE_STRIP)
        marker = new ros.visualization.Markers.LineStripMarker(this.vm); 
      else if (marker_type == ros.visualization.Markers.MarkerTypes.TRIANGLE_LIST)
        marker = new ros.visualization.Markers.TriangleListMarker(this.vm);
      else if (marker_type == ros.visualization.Markers.MarkerTypes.TEXT_VIEW_FACING)
        marker = new ros.visualization.Markers.ViewfacingTextMarker(this.vm);
      else if (marker_type == ros.visualization.Markers.MarkerTypes.MESH_RESOURCE)
        marker = new ros.visualization.Markers.MeshMarker(this.vm);
      else {
		    ros_error("Marker type " + marker_type + " is currently not supported.");
		    return false;
		  }
//      marker.setPickable(true);
		  // update marker from message
		  marker.updateFromMessage(marker_msg);
      marker.setCumMatrix();
		  node_id = this.vm.scene_viewer.addNode(marker);
		  this.markerNodes.insert(marker_id, node_id);

	  }
	  
	  else {
	    ros_error("Marker action " + marker_action + " is currently not supported.");
	    return false;
	  }
    
    return true;
  },
  
  getMarkerStringID: function(message) 
  {
    return message.ns + "/" + message.id;
  },
  
});
