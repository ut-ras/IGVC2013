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


ros.visualization.InteractiveMarkers.Tool = ros.visualization.InteractiveMarkers.Tool || {};

ros.visualization.InteractiveMarkers.Tool.autoComplete = function(control, scale) 
  {
//    control.orientation.normalize();
    var INT_MODE = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.INTERACTION_MODE;
    if(control.description == "")
    {
      switch(control.interaction_mode)
      {
        case INT_MODE.NONE:
          break;
        case INT_MODE.MOVE_AXIS:
          control.description = "Drag to move along the axis.";
          break;
        case INT_MODE.MOVE_PLANE:
          control.description = "Drag to move in the plane.";
          break;
        case INT_MODE.ROTATE_AXIS:
          control.description = "Drag to rotate.";
          break;
        case INT_MODE.MOVE_ROATE:
          control.description = "Drag to rotate and move.";
          break;
        case INT_MODE.BUTTON:
          control.description = "Click to activate.";
          break;
        case INT_MODE.MENU:
          control.description = "Menu";
          break;
        default:
          break;
      }
    }

    if(control.markers.length == 0)
    {
      switch(control.interaction_mode)
      {
        case INT_MODE.NONE:
          break;
        case INT_MODE.MOVE_AXIS:
          ros.visualization.InteractiveMarkers.Tool.makeArrow(control,scale,1.0);
          ros.visualization.InteractiveMarkers.Tool.makeArrow(control,scale,-1.0);
          break;
        case INT_MODE.MOVE_PLANE: 
        case INT_MODE.ROTATE_AXIS:
        case INT_MODE.MOVE_ROTATE:
          ros.visualization.InteractiveMarkers.Tool.makeDisc(control,scale,0.3);
//          ros_debug('disc is not implemented yet');
          break;
        case INT_MODE.BUTTON:
          ros_debug('Interactive Marker Button is not implemented yet');
          break;

        case INT_MODE.MENU:
          ros_debug('Interactive Marker Menu is not implemented yet');
          break;
        default:
          ros_debug('Interactive Marker Auto complete Error');
          break;
      }
    }
  };

ros.visualization.InteractiveMarkers.Tool.makeTitle = function(scale,text)
{
  var type = ros.visualization.Markers.MarkerTypes.TEXT_VIEW_FACING;
  var color = {r:0.1,g:0.1,b:0.1,a:1.0};
  var marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale,color,{w:1,x:0,y:0,z:0});

  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.15;
  marker.scale.z = scale * 2;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1;

  marker.pose.position.z = scale * 1;
  marker.text = text;

  var control = {};
  control.interaction_mode = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.INTERACTION_MODE.NONE;
  control.orientation_mode = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.ORIENTATION_MODE.FIXED;
  control.orientation = {};
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 1;
  control.always_visible = true;
  control.markers = [];
  control.markers.push(marker);

  return control;
};

ros.visualization.InteractiveMarkers.Tool.makeArrow = function(control, scale, pos)
  {
    var type = ros.visualization.Markers.MarkerTypes.ARROW;
    var color = {};
    var marker;

    if(control.orientation.x == 0 && control.orientation.y == 0 &&control.orientation.z == 0)
      color.r = 1;
    else 
      color.r = Math.abs(control.orientation.x);
    color.g = Math.abs(control.orientation.z);
    color.b = Math.abs(control.orientation.y);
    color.a = 1.0;

    marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale, color,control.orientation);

    marker.scale.x = scale * 0.6;
    marker.scale.y = scale * 1;
    marker.scale.z = scale * 0.4;

    var dist = Math.abs(pos);
    var dir = pos > 0 ? 1: -1;

    var inner = 0.5 * dist;
    var outer = inner - (-0.4);

    marker.points = [];
    marker.points.push(new ros.math.Vector3(dir * scale * inner, 0,0));
    marker.points.push(new ros.math.Vector3(dir * scale * outer, 0,0));

    control.markers.push(marker);
};

ros.visualization.InteractiveMarkers.Tool.makeDisc = function(control, scale, width)
{
  var type = ros.visualization.Markers.MarkerTypes.TRIANGLE_LIST;
  
  var steps = 18;
  var circle1 = [];
  var circle2 = [];
  var v1;
  var v2;

  var color = {};
  color.r = Math.abs(control.orientation.x);
  color.g = Math.abs(control.orientation.y);
  color.b = Math.abs(control.orientation.z);
  color.a = 1;

  
  for(var i = 0; i < steps; i++)
  {
    var a = (i / steps) * Math.PI * 2.0;

    v1 = new ros.math.Vector3();
    v2 = new ros.math.Vector3();

    v1.y = 0.5 * Math.cos(a);
    v1.z = 0.5 * Math.sin(a);

    v2.y = (1 - (-width)) * v1.y;
    v2.z = (1 - (-width)) * v1.z;

    circle1.push(v1);
    circle2.push(v2);
  }

  var Mode = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.INTERACTION_MODE;

  switch(control.interaction_mode)
  {
    case Mode.ROTATE_AXIS:
      for(var i = 0 ; i < steps; i++)
      {
        var marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale, color,control.orientation);
        var i1 = i;
        var i2 = (i - (-1)) % steps;
        var i3 = (i - (-2)) % steps;

        if(color.r == 0 && color.g == 0 && color.b == 0)
        {
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
          marker.color.a = 1;
        }

        var t = 0.9 * 0.6 * (i%2);
        marker.color.r = marker.color.r * t;
        marker.color.g = marker.color.g * t;
        marker.color.b = marker.color.b * t;
        marker.color.a = marker.color.a * t;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i2];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle1[i2];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle2[i3];

        control.markers.push(marker);
      }
      break;
    case Mode.MOVE_ROTATE:
      for(var i = 0; i < steps - 1; i+=2)
      {
        var marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale,color,control.orientation);
        var i1 = i;
        var i2 = (i - (-1)) % steps;
        var i3 = (i - (-2)) % steps;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i2];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle1[i2];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle2[i3];

        if(color.r == 0 && color.g == 0 && color.b == 0)
        {
          marker.color.r = 0.5;
          marker.color.g = 0.1;
          marker.color.b = 0.1;
        }
        control.markers.push(marker);
        
        var marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale, color,control.orientation);

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i2];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle1[i2];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle2[i3];
        
        control.markers.push(marker);
      }
      break;
    default:
      for(var i = 0; i < steps; i++) 
      {
        var marker = ros.visualization.InteractiveMarkers.Tool.createBaseMarker(type, scale, color,control.orientation);
        var i1 = i;
        var i2 = (i+1) % steps;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i1];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle1[i1];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle2[i2];

        if(color.r == 0 && color.g  == 0 && color.b == 0)
        {
          marker.color.r = 0.5;
          marker.color.g = 0.1;
          marker.color.b = 0.1;
        }

        control.markers.push(marker);
      }
  }
};

ros.visualization.InteractiveMarkers.Tool.createBaseMarker = function(type,scale, color,ori)
{
  var marker = {};

  marker.lifetime = 0;
  marker.header = {};
  marker.header.frame_id = "/";

  marker.pose = {};
  marker.pose.position = new ros.math.Vector3();
  marker.pose.orientation = new ros.math.Quaternion(ori.w,ori.x,ori.y,ori.z);
//  marker.orientation = new ros.math.Quaternion(ori.w,ori.x,ori.y,ori.z);

  marker.type = type;
  marker.scale = {};
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color = {};
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0;

  marker.color.r = marker.color.r == 0?0.1:marker.color.r;
  marker.color.g = marker.color.g == 0?0.1:marker.color.g;
  marker.color.b = marker.color.b == 0?0.1:marker.color.b;

//  marker.colors = [];
  marker.points = [];

  return marker;
};


ros.visualization.InteractiveMarkers.MENU_COMMAND_TYPE = {
              "FEEDBACK" : 0, "ROSRUN" : 1, "ROSLAUNCH" : 2};

