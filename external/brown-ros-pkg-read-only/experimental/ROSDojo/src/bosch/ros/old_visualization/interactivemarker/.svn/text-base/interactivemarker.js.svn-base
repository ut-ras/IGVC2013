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

ros.visualization.InteractiveMarkers = ros.visualization.InteractiveMarkers || {};
/**
 * Class that handles  interactive markers.
 * @class
 * @augments ros.visualization.SceneNode
 */
ros.visualization.InteractiveMarkers.InteractiveMarker = ros.visualization.SceneNode.extend(
/** @lends ros.visualization.InteractiveMarkers.InteractiveMarker# */
{
	/**
	 * Initialization function
	 * 
	 * @param vm visualization manager
	 * 
	 */	
  init: function(vm) {
    this._super(vm);
    this.name = "InteractiveMarker";
    this.description = null;
    this.scale = null;
    this.menu = new Array();
    this.controls = this.children;
    this.setPickable(true);
    this.feedbacktopic ="";
    this.feedbacktopictype = 'visualization_msgs/InteractiveMarkerFeedback';
    this.lastPose = null;
  },
  
  updateFromMessage: function(msg)
  {
    this.name = msg.name;
    this.description = msg.description;
    this.setFrame(msg.header.frame_id);
    this.setScale([1,1,1]);
    this.setPose(msg.pose);

    this.lastPose = msg.pose;

    if(msg.scale == 0)
      msg.scale = 1;

    
    if(this.description != "") {
      var title = ros.visualization.InteractiveMarkers.Tool.makeTitle(msg.scale, this.description);

      var title_control = new ros.visualization.InteractiveMarkers.InteractiveMarkerControl(this.vm, this);
      title_control.updateFromMessage(title);
      this.controls.push(title_control);
      title_control.setCumMatrix(); 
    }

    for(var c in msg.controls)
    {
      var control = new ros.visualization.InteractiveMarkers.InteractiveMarkerControl(this.vm, this);
      ros.visualization.InteractiveMarkers.Tool.autoComplete(msg.controls[c],msg.scale);
      control.updateFromMessage(msg.controls[c]);
      this.controls.push(control);
      control.setCumMatrix();
    }

    this.createMenu(msg.menu_entries);
  },

  /**
   * Creates menu that will appear on right click of interactive marker
   * @param menu_entries array of objects describing the commands provided in the menu
   */
  createMenu : function(menu_entries)
  {
    var menu = {};
    
    var itemArray = [];
    var that = this;
    
    for(var i in menu_entries)
    {
      var item = {}
      item.type = RightContext.TYPE_MENU;
      item.text = menu_entries[i].title;
      item.onclick = function() { that.menuEvent(this)};
      item.command_type = menu_entries[i].command_type;
      item.command = menu_entries[i].command;
      item.id = menu_entries[i].id;
      itemArray.push(item);
    }

    menu.attributes = "";
    menu.items = itemArray;

    this.menu = menu;
  },

   menuEvent : function(item)
   {
      var command_type = item.getAttribute("command_type"); 
      var id = item.getAttribute("id"); 
      var feedback = this.feedbackMsg;
      feedback.event_type  = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;
      feedback.menu_entry_id = parseInt(id);

      this.publishFeedback(feedback);
   },

  updateFromPoseMessage: function(pose)
  {
    if(this.mouseDragging)
    {
      this.lastPose = pose.pose;
    }
    else {
      this.lastPose = pose.pose;
      this.poseUpdate();
    }
  },

  mouseEvent : function(control, control_name, pickInfo, mouseButton, type)
  {
    var feedback = new ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback();

    feedback.client_id = "wviz";
    feedback.event_type = type;
    feedback.marker_name = this.name;
    feedback.control_name = control_name;
    
    feedback.mouse_point_valid = pickInfo.valid;
    if(pickInfo.valid)
    {
      var m = this.vm.scene_viewer.lookupTransform(this.frame_id); 
      var matrix = sglInverseM4(m);

      var refPoint = ros.visualization.transformPoint(matrix, pickInfo.worldIntersectionPosition);        
      feedback.mouse_point = {x:refPoint[0], y:refPoint[1], z:refPoint[2]};
    }
    
    if(mouseButton[0])
    { 
      this.publishFeedback(feedback);

      if(type == ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MOUSE_UP) 
      {
        this.poseUpdate();
        this.mouseDragging = false;
        return false;
      }
      else {
        this.mouseDragging = true;
        return true;
      }
    }
    else if(mouseButton[2]){
      this.feedbackMsg = feedback;
      RightContext.setMenu(this.menu);
      return true;
    }

    return false;
  },

  poseUpdate : function()
  {
    this.setPose(this.lastPose);
  },


  publishPose: function(name)
  {
    var feedback = new ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback();

    feedback.client_id = "wviz";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.POSE_UPDATE;
    feedback.marker_name = this.name;
    feedback.control_name = name;

    for(var c in this.controls)
    {
      var control = this.controls[c];
      control.setCumMatrix();
    }

    this.publishFeedback(feedback);
  },

  /**
   * Publish interactive marker feedback to server
   */
  publishFeedback: function(feedback)
  {
    var rosNode = this.vm.node;

    feedback.header = new ros.roslib.Header();
    feedback.header.frame_id = this.frame_id;
    feedback.header.stamp = (new ros.Time).now(); 

    feedback.pose = {};
    feedback.pose.position = this.position;
    feedback.pose.orientation = this.orientation;

    rosNode.publish(this.feedbacktopic,this.feedbacktopictype,ros.json(feedback));
  },

  getSize : function()
  {
    return this.scale[0];
  },

});

ros.include('visualization/interactivemarker/interactivemarkercontrol');
ros.include('visualization/interactivemarker/interactivemarkerupdate');
ros.include('visualization/interactivemarker/interactivemarkerfeedback');
ros.include('visualization/interactivemarker/tool');
