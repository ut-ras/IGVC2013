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

ros.visualization.InteractiveMarkers.Update = ros.visualization.InteractiveMarkers.InteractiveMarkerUpdate || {};

/**
 * Class that handles user interaction of interactive markers.
 * @class
 * @augments Class
 */

ros.visualization.InteractiveMarkers.InteractiveMarkerUpdate = Class.extend({
	/**
	 * Initialization function
	 * 
	 * @param vm visualization manager
	 * @param manager interactive marker manager
	 * 
	 */	
  
  init: function(vm,manager) {
    this.vm = vm;
    this.manager = manager;
    this.topic;
    this.server_id = "";
    this.seq_num = 0;
    this.iMarkerNodes = new ros.Map();
    this.feedbacktoppic = "";
  },

  /**
	 * Function that subscribes to interactive marker topic on the server
	 * A call to this function saves two topic types the first is topic specified in imarker_topic and to the full topic specified as im_marker_topic+full
	 * This function than subscribes to the full_topic
	 * 
	 * @param imarker_topic
	 * 
	 */	
  subscribeInteractiveMarker: function(imarker_topic) {
    // subscribe to interactive marker
    var that = this;
    var server_id = this.getStringID(imarker_topic);
    this.full_topic = imarker_topic + '_full';
    this.topic = imarker_topic;
    this.server_id = server_id;
    this.feedbacktopic = server_id + '/feedback';
    this.vm.node.subscribe(this.full_topic, function(msg) { that.receiveInteractiveMarkerMessage(msg);});
  },

  /**
   * Function that handles incoming interactive marker messages
   * 
   * There are 3 types of interactive marker messages: KEEP_ALIVE, UPDATE, and INIT
   * On receipt of an INIT message the markers are updated and the client unsubscribes from the full_topic and subscribes to the topic (see details of subscribeInteractiveMarker)
   * On receipt of an UPDATE message the markers and poses are updated and markers are erased if necessary
   * 
   * @param msg 
   */
  receiveInteractiveMarkerMessage: function(msg) {
    var types = ros.visualization.InteractiveMarkers.InteractiveMarkerUpdate.Types;

    var that = this;
    var server_id = this.server_id;

    if(msg.type == null || msg.type == "undefined") // init msg
    {
       ros_debug("server id = " + server_id + ": Init!");  
       this.updateMarkers(msg.markers);                                                                     
       this.vm.node.unsubscribe(this.full_topic);
       this.vm.node.subscribe(this.topic, function(msg) { that.receiveInteractiveMarkerMessage(msg);});
       this.vm.scene_viewer.receivedInteractiveMarker();
       return;
    }

    switch(msg.type) 
    {
      case types.KEEP_ALIVE:
//        ros_debug("server id = " + server_id + ": Connection is alive!");  
      break;
      case types.UPDATE:
        ros_debug("server id = " + server_id + " seq = " + msg.seq_num + ": Update!");  
        this.updateMarkers(msg.markers);
        this.updatePoses(msg.poses);
        if(msg.erases.length != 0)
          this.eraseMarkers(msg.erases);
      break;
      case types.INIT:
        ros_debug("server id = " + server_id + ": Init!");  
        this.updateMarkers(msg.markers);
        this.vm.node.unsubscribe(this.full_topic);
        this.vm.node.subscribe(this.topic, function(msg) { that.receiveInteractiveMarkerMessage(msg);});
      break;
      default:
        this.updateMarkers(msg.markers);
    }

    this.vm.scene_viewer.receivedInteractiveMarker();
  },

  /**
   * Updates all markers in array
   * 
   * 
   * @param markers array of marker messages
   */
  updateMarkers: function(markers) {
    for(var i in markers)
      this.updateMarker(markers[i]);

  },
  
  /**
   * Update marker from message
   * 
   * 
   * @param marker_msg message containing marker info
   */
  updateMarker: function(marker_msg) {
    var marker = this.iMarkerNodes.find(marker_msg.name); 
    
    if(marker != null) {
      this.vm.scene_viewer.removeNode(marker.sceneview_node_id);
      this.iMarkerNodes.remove(marker.name);
    }

    marker = new ros.visualization.InteractiveMarkers.InteractiveMarker(this.vm);
    marker.feedbacktopic = this.feedbacktopic;
    marker.updateFromMessage(marker_msg);  
    node_id = this.vm.scene_viewer.addNode(marker);
    marker.sceneview_node_id = node_id;
    this.iMarkerNodes.insert(marker.name, marker);
  },

  /**
   * Updates all poses from msg array
   * 
   * 
   * @param poses array of pose messages
   */
  updatePoses: function(poses) {
    for(var i in poses)
      this.updatePose(poses[i]);
  },

  /**
   * Update pose from message
   * 
   * 
   * @param pose message containing pose info
   */
  updatePose: function(pose) {
    var marker = this.iMarkerNodes.find(pose.name);

    if(marker)
      marker.updateFromPoseMessage(pose);
  },
  
  /**
   * Erases specified markers
   * 
   * 
   * @param erase_list list of markers to erase
   */
  eraseMarkers: function(erase_list) {
    var name;
    var node;
    for(var i in erase_list)
    {
      name = erase_list[i];
      node = this.iMarkerNodes.find(name);

      if(node != null) {
        this.vm.scene_viewer.removeNode(node.sceneview_node_id);
        this.iMarkerNodes.remove(name);
      }
    }
  },

  getStringID : function(server_id)
  {
    var out = server_id.split("/")[1];
    return out;
  }
});

ros.visualization.InteractiveMarkers.InteractiveMarkerUpdate.Types = {"INIT" : 2, "UPDATE" : 1, "KEEP_ALIVE" : 0};
