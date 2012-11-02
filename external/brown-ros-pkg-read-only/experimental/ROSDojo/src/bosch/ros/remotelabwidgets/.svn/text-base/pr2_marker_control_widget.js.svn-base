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

ros.widgets.Pr2_Marker_Control_Widget = Class.extend({
  init : function(node, divID, vm) {
    this.node = node;
    this.divID = divID;
    this.vm = vm;

    this.divbuttonID = 'buttondiv';

    this.setButtonText();
    this.create_divHTML();
    this.setCallbacks();

    this.feedbacktopic = "pr2_marker_control/feedback";
    this.feedbacktopictype = 'visualization_msgs/InteractiveMarkerFeedback';

    this.l_gripper = 2;
    this.r_gripper = 2;

    this.l_hand = 2;
    this.r_hand = 2;

  },

  setButtonText : function()
  {
    this.initButtonID = "init_button";
    this.initButtonTitle ="Initialize Markers";
    this.initButton = this.createButton(this.initButtonID, this.initButtonTitle);

    this.headButtonID = "head_target";
    this.headButtonTitle = "Head Target On/Off";
    this.headButton = this.createButton(this.headButtonID,this.headButtonTitle);

    this.lefthandButtonID = "left_hand_click";
    this.leftButtonTitle = "Left Hand On/Off";
    this.leftHandButton = this.createButton(this.lefthandButtonID,this.leftButtonTitle);

    this.lefthandmodeButtonID = "left_hand_switch";
    this.leftmodeButtonTitle = "Left Hand Mode Switch";
    this.lefthandModeButton = this.createButton(this.lefthandmodeButtonID,this.leftmodeButtonTitle);

    this.leftGripperId = "left_gripper_click";
    this.leftGripperTitle = "Left Gripper Open/Close";
    this.leftGripperButton = this.createButton(this.leftGripperId, this.leftGripperTitle);

    this.righthandButtonID = "right_hand_click";
    this.righthandButtonTitle = "Right Hand On/Off";
    this.righthandButton = this.createButton(this.righthandButtonID,this.righthandButtonTitle);

    this.righthandmodeButtonID = "right_hand_switch";
    this.righthandmodeButtonTitle = "Right Hand Mode Switch";
    this.righthandModeButton = this.createButton(this.righthandmodeButtonID,this.righthandmodeButtonTitle);

    this.rightGripperId = "right_gripper_click";
    this.rightGripperTitle = "Right Gripper Open/Close";
    this.rightGripperButton = this.createButton(this.rightGripperId, this.rightGripperTitle);

    this.baseMoveId = "base_move_click";
    this.baseMoveTitle = "Base Move On/Off";
    this.baseMoveButton = this.createButton(this.baseMoveId,this.baseMoveTitle);
  },

  create_divHTML : function()
  {
     div_text = "<div id=\"" + this.divbuttonID + "\">"; 
     div_text = div_text + this.initButton;
     div_text = div_text + this.headButton;
     div_text = div_text + this.leftHandButton;
     div_text = div_text + this.lefthandModeButton;
     div_text = div_text + this.leftGripperButton;
     div_text = div_text + this.righthandButton;
     div_text = div_text + this.righthandModeButton;
     div_text = div_text + this.rightGripperButton;
     div_text = div_text + this.baseMoveButton;
     div_text = div_text + "</div>";

     $('#'+this.divID).html(div_text);
  },

  createButton : function(id, title)
  {
    return " <p><button type=\"button\", id=\"" + id + "\" style=\"width: 200px;\"> " + title + "</button></p>";
  },

  setCallbacks : function()
  {
    var that = this;
    jQuery('#init_button').click(function(e) {
      that.initClick();
    });

    jQuery('#head_target').click(function(e) {
      that.headClick();
    });

    jQuery('#left_hand_click').click(function(e) {
      that.leftHandClick();
    });

    jQuery('#left_hand_switch').click(function(e) {
      that.leftHandSwitch();
    });

    jQuery('#left_gripper_click').click(function(e) {
      that.leftGripperClick();
    });

    jQuery('#right_hand_click').click(function(e) {
      that.rightHandClick();
    });

    jQuery('#right_hand_switch').click(function(e) {
      that.rightHandSwitch();
    });
    jQuery('#right_gripper_click').click(function(e) {
      that.rightGripperClick();
    });

    jQuery('#base_move_click').click(function(e) {
      that.baseMoveClick();
    });                                                  
  },

  initClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/torso_lift_link";                                                         
    feedback.marker_name = "PR2 Marker Control Root";                                                              
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.BUTTON_CLICK;    

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },
  headClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/head_tilt_link";                                                         
    feedback.marker_name = "head_tilt_link";                                                              
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;    
    feedback.menu_entry_id = 1;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  leftHandClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/l_gripper_palm_link";
    feedback.marker_name = "l_gripper_palm_link";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.BUTTON_CLICK;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  leftHandSwitch : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/l_gripper_control";
    feedback.marker_name = "l_gripper_control";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;
    feedback.menu_entry_id = this.l_hand;

    this.l_hand = this.l_hand == 2?3:2;
    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },


  leftGripperClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/l_gripper_palm_link";
    feedback.marker_name = "l_gripper_palm_link";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;
    feedback.menu_entry_id = this.l_gripper;

    this.l_gripper = this.l_gripper ==1?2:1;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  rightHandClick :  function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/r_gripper_palm_link";
    feedback.marker_name = "r_gripper_palm_link";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.BUTTON_CLICK;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  rightHandSwitch : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/r_gripper_control";
    feedback.marker_name = "r_gripper_control";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;
    feedback.menu_entry_id = this.r_hand;

    this.r_hand = this.r_hand == 2?3:2;
    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  rightGripperClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/r_gripper_palm_link";
    feedback.marker_name = "r_gripper_palm_link";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.MENU_SELECT;
    feedback.menu_entry_id = this.r_gripper;

    this.r_gripper = this.r_gripper ==1?2:1;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },

  baseMoveClick : function()
  {
    var feedback = this.createbaseFeedback();
    feedback.header.frame_id = "/base_link";
    feedback.marker_name = "base_link";
    feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.BUTTON_CLICK;

    this.node.publish(this.feedbacktopic, this.feedbacktopictype,ros.json(feedback));
  },
  
  createbaseFeedback : function()
  {
    var feedback = new ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback();

    feedback.header = new ros.roslib.Header();
    feedback.frame_id = '';
    feedback.header.stamp = (new ros.Time).now();
    feedback.client_id = "wviz";
    feedback.control_name = '';

    feedback.pose ={};
    feedback.pose.position = {x:0,y:0,z:0};
    feedback.orientation = new ros.math.Quaternion();

    return feedback;
  },

});
