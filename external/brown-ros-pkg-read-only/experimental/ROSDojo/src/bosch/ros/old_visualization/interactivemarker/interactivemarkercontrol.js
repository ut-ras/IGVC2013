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
/**
 * Class that handles user interaction of interactive markers.
 * @class
 * @augments ros.visualization.SceneNode
 */
ros.visualization.InteractiveMarkers.InteractiveMarkerControl = ros.visualization.SceneNode.extend(
/** @lends ros.visualization.InteractiveMarkers.InteractiveMarkerControl# */
{
 
/**
 * Initialization function
 * 
 * @param vm visualization manager
 * @param parent
 * 
 */	
  init: function(vm, parent)
  {
    this._super(vm); 
    this.name = "";
    this.orientation_mode = 0;
    this.interaction_mode = 0;
    this.markers = this.children;
    this.independent_marker_orientation = false;
    this.description = "";
    this.setPickable(false);
    this.setEnable(true);

    this.referenceOrientation = new ros.math.Quaternion();
    this.controlOrientation = new ros.math.Quaternion();
    this.viewOrientation = new ros.math.Quaternion();

    this.parent = parent;

  },

  /**
   * Function to enable/disable markers.  If always_visible is set enabled will always be true no matter what user input is.
   */
  
  setEnable : function(enabled)
  {
    this.enabled = enabled;
    if(this.always_visible)
      this.enabled = true;
  },

  /**
   * Update interactive markers from message
   */
		  
  updateFromMessage: function(msg)
  {
    this.independent_marker_orientation = msg.independent_marker_orientation;
    this.description = msg.description;
    this.name = msg.name;
    this.always_visible = msg.always_visible;

    this.orientation_mode = msg.orientation_mode;
    this.interaction_mode = msg.interaction_mode;

    this.controlOrientation = new ros.math.Quaternion(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    this.controlFrameOrientation = new ros.math.Quaternion();
    this.initialParentOrientation = this.parent.getOrientation();

    if(this.interaction_mode > 0) {
      this.setPickable(false);
      this.setEnable(this.always_visible);
    }
    else {
      this.setEnable(true);
    }

    this.rotationAngle = 0.0;
    this.createMarkers(msg.markers);
    this.setOrientationMode(msg.orientation);
    this.setCumMatrix();
  },

  /**
   * Create markers with specified type
   */
  createMarkers: function(markers) 
  {
    var MarkerTypes = ros.visualization.Markers.MarkerTypes;
    for(var m in markers)
    { 
      var marker = null;
      switch(markers[m].type) {
        case MarkerTypes.ARROW:
          marker = new ros.visualization.Markers.ArrowMarker(this.vm);
          break;
        case MarkerTypes.CUBE:
          marker = new ros.visualization.Markers.CubeMarker(this.vm);  
          break;
        case MarkerTypes.SPHERE:
          marker = new ros.visualization.Markers.SphereMarker(this.vm); 
          break;
        case MarkerTypes.CYLINDER:
          marker = new ros.visualization.Markers.CylinderMarker(this.vm); 
          break;
        case MarkerTypes.SPHERE_LIST:
          marker = new ros.visualization.Markers.PointsMarker(this.vm);
        case MarkerTypes.POINTS:
		      marker = new ros.visualization.Markers.PointsMarker(this.vm); 
          break;
        case MarkerTypes.LINE_LIST:
          marker = new ros.visualization.Markers.LineListMarker(this.vm); 
          break;
        case MarkerTypes.LINE_STRIP:
          marker = new ros.visualization.Markers.LineStripMarker(this.vm); 
          break;
        case MarkerTypes.TRIANGLE_LIST:
          marker = new ros.visualization.Markers.TriangleListMarker(this.vm); 
          break;
        case MarkerTypes.TEXT_VIEW_FACING:
          marker = new ros.visualization.Markers.ViewfacingTextMarker(this.vm);
          break;
        case MarkerTypes.MESH_RESOURCE:
          marker = new ros.visualization.Markers.MeshMarker(this.vm);
          break;
        default:
          ros_error("Marker type " + markers[m].type + " is currently not supported.");
          continue;
//          return false;
      }
      marker.updateFromMessage(markers[m]);
      marker.setParent(this);
      marker.setPickable(true);
      marker.load();
      this.markers.push(marker);
    }
  },

  setOrientationMode : function(msg_ori)
  {
    var MODE = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.ORIENTATION_MODE;
    
    switch(this.orientation_mode)
    {
      case MODE.INHERIT:
        this.referenceOrientation = this.parent.getOrientation();
        this.orientation = new ros.math.Quaternion();
        break;
      case MODE.FIXED:
        this.referenceOrientation = this.initialParentOrientation;
        this.orientation = new ros.math.Quaternion();
        break;
      case MODE.VIEW_FACING:
        this.referenceOrientation = this.controlFrameOrientation;
        this.orientation = new ros.math.Quaternion();
        this.vm.scene_viewer.addListener(this);

        break;
      default:
        ros_error('InteractiveMarker control : Wrong Mode Type = ' + this.orientation_mode);
        break;
    }
  },

  setCumMatrix : function()
  {
    var MODE = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.ORIENTATION_MODE;
    var pmatrix = sglIdentityM4();
    var pscale = sglScalingM4V(this.parent.scale);

    var matrix = sglIdentityM4();
    var translation = sglTranslationM4V([this.position.x, this.position.y, this.position.z]);
    var rotation = sglGetQuatRotationM4([this.orientation.x, this.orientation.y, this.orientation.z, this.orientation.w]);
    matrix = sglMulM4(matrix, translation);
    matrix = sglMulM4(matrix, rotation);

    switch(this.orientation_mode)
    {
      case MODE.INHERIT:
        pmatrix = sglMulM4(pmatrix, this.parent.matrix);
        pmatrix = sglMulM4(pmatrix, pscale);
        this.matrix = sglMulM4(pmatrix, matrix);
        break;
      case MODE.FIXED:
        var axis = this.referenceOrientation.multiplyVector(this.controlOrientation.xAxis());
        var quat = new ros.math.Quaternion();

        axis.normalise();
        quat.fromAngleAxis(this.rotationAngle,axis);
        pmatrix = ros.visualization.getPoseMatrix(this.parent.getPosition(),quat);
        pmatrix = sglMulM4(pmatrix, pscale);
        this.matrix = sglMulM4(pmatrix, matrix);
        break;
      case MODE.VIEW_FACING:

        this.setControlFrameOrientation();

        if(this.independent_marker_orientation)
        {
          pmatrix = sglMulM4(pmatrix, this.parent.matrix);
          pmatrix = sglMulM4(pmatrix, pscale);
          this.matrix = sglMulM4(pmatrix, matrix);
        }
        else {
          pmatrix = sglMulM4(pmatrix, this.parent.matrix);
          pmatrix = sglMulM4(pmatrix, pscale);
          var mat = ros.visualization.getPoseMatrix(this.position,this.viewOrientation);
          this.matrix = sglMulM4(pmatrix, mat);
        }
        break;
      default:
        ros_error('InteractiveMarkerControl : Wrong mode type = ' + this.orientation_mode);
        break;
    }

    for(var c in this.children)
    {
      this.children[c].setCumMatrix();
    }
  },

  setControlFrameOrientation : function()
  {
    var ori = this.getOrientation();
    var pmatrix = sglIdentityM4();
    var pscale = sglScalingM4V(this.parent.scale);
    pmatrix = sglMulM4(pmatrix, this.parent.matrix);
    pmatrix = sglMulM4(pmatrix, pscale);

    var viewmatrix = this.vm.scene_viewer.getViewMatrix();

    var vmatrix = ros.visualization.getRotationMat(viewmatrix);
    var mmatrix = ros.visualization.getRotationMat(pmatrix);

    vmatrix = sglInverseM4(vmatrix);
    mmatrix = sglInverseM4(mmatrix); 

    var neg_z = ros.math.Vector3.NEGATIVE_UNIT_Z;
    var lookvector_sgl = sglMulM4V3(vmatrix,neg_z.toArray(),0.0);
    lookvector_sgl = sglMulM4V3(mmatrix, lookvector_sgl, 0.0);
    var lookvector = new ros.math.Vector3(lookvector_sgl[0], lookvector_sgl[1], lookvector_sgl[2]);

    var pos_y = ros.math.Vector3.UNIT_Y;
    var upvector_sgl = sglMulM4V3(vmatrix, pos_y.toArray(),0.0);
    upvector_sgl = sglMulM4V3(mmatrix, upvector_sgl, 0.0);
    var upvector = new ros.math.Vector3(upvector_sgl[0],upvector_sgl[1], upvector_sgl[2]);

    var xaxis = ori.xAxis();
    var x_quat= xaxis.getRotationTo(lookvector);
    ori.multiplyQuat(x_quat);

    var z_axis = ori.yAxis();
    var z_quat = z_axis.getRotationTo(upvector);
    ori.multiplyQuat(z_quat);

    var parentorientation = this.parent.getOrientation();
    var childorientation = ori.copy();
    childorientation.multiplyQuat(parentorientation);

    var quat = new ros.math.Quaternion();
    var axis = ori.xAxis();
    axis.normalise();
    quat.fromAngleAxis(this.rotationAngle,axis);

    ori.multiplyQuat(quat);
    this.viewOrientation = ori.copy();
    this.controlFrameOrientation = childorientation.copy();
    this.controlFrameOrientation.normalise();
  },
  
  setReferenceOrientation : function()
  {
    var MODE = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.ORIENTATION_MODE;
    
    switch(this.orientation_mode)
    {
      case MODE.INHERIT:
        this.referenceOrientation = this.parent.getOrientation();
        break;
      case MODE.FIXED:
        this.referenceOrientation = this.initialParentOrientation.copy();
        break;
      case MODE.VIEW_FACING:
        this.setControlFrameOrientation();
        this.referenceOrientation = this.controlFrameOrientation.copy();
        break;
      default:
        ros_error('InteractiveMarker control : Wrong Mode Type = ' + this.orientation_mode);
        break;
    }
  },

 /**
  * Handle mouse events for interacting with makers including clicking and axis moves
  */
  mouseEvent : function(mouse_ray, last_mouse_ray,mouseButtons, mode)
  {
    var MODE = ros.visualization.InteractiveMarkers.InteractiveMarkerControl.INTERACTION_MODE;
    var feedback = new ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback();

    var current_ray = new ros.geometry.Ray(mouse_ray.far[0],mouse_ray.far[1],mouse_ray.far[2],
                                           mouse_ray.near[0],mouse_ray.near[1],mouse_ray.near[2]);

    var last_ray = new ros.geometry.Ray(last_mouse_ray.far[0],last_mouse_ray.far[1],last_mouse_ray.far[2],
                                        last_mouse_ray.near[0],last_mouse_ray.near[1],last_mouse_ray.near[2]);

    this.setReferenceOrientation();

    switch(this.interaction_mode)
    {
      case MODE.BUTTON:
        if(mouseButtons[0] && mode == 2) {
          feedback.client_id = "wviz";
          feedback.event_type = ros.visualization.InteractiveMarkers.InteractiveMarkerFeedback.Types.BUTTON_CLICK;
          feedback.control_name = this.name;
          feedback.marker_name = this.parent.name;
          this.parent.publishFeedback(feedback);
        }
        break;
      case MODE.MOVE_AXIS:
        if(mouseButtons[0] && mode == 1)
        {
          var last_pos = this.getClosestPosOnAxis(last_ray);
          var pos = this.getClosestPosOnAxis(current_ray);
          var delta = pos - last_pos;
          var translate_delta = this.referenceOrientation.multiplyVector(this.controlOrientation.xAxis());
          translate_delta.multiplyScalar(delta);
          
          this.parent.translate(translate_delta);
          this.parent.publishPose(this.name); 
        }
        break;
      case MODE.MOVE_PLANE:
        if(mouseButtons[0] && mode == 1)
        {
          var translate_delta = this.movePlane(current_ray, last_ray);

          this.parent.translate(translate_delta);
          this.parent.publishPose(this.name);
        }
        break;
      case MODE.MOVE_ROTATE:
        if(mouseButtons[0] && mode == 1)
        {
          var quat_delta = this.rotateAxis(current_ray, last_ray);
          var translate_delta = this.followMouse(current_ray, this.children[0].getSize() * 0.8);

          if(quat_delta != null)
            this.parent.rotate(quat_delta);
          this.parent.translate(translate_delta);
          this.parent.publishPose(this.name);
        }
        break;
      case MODE.ROTATE_AXIS:
        if(mouseButtons[0] && mode == 1)
        {
          var quat_delta =  this.rotateAxis(current_ray, last_ray);
          if(quat_delta != null) {
            this.parent.rotate(quat_delta);
            this.parent.publishPose(this.name);
          }
          break;
        }
      case MODE.MENU:
        break;
      default:
        ros_debug('mode : ' + this.interaction_mode + ' is not implemented yet');
        log('mode : ' + this.interaction_mode + ' is not implemented yet');
        break;
    }
  },

  followMouse : function(ray, max_dist)
  {
    var current = this.intersectYzPlane(ray);
    var diff = current.i3d.subtract(this.parent.getPosition());

    if(diff.length() > max_dist)
    {
      var length = diff.normalise();
      var t_delta = diff.multiplyScalar(length - max_dist);
      return t_delta;
    }

    return new ros.math.Vector3();
  },

  movePlane : function(ray, old_ray)
  {
    var last, current;

    current = this.intersectYzPlane(ray);
    last = this.intersectYzPlane(old_ray);
    
    var t_delta = current.i3d.subtract(last.i3d);

    return t_delta;

  },

  rotateAxis : function(ray, old_ray)
  {
    var last, current;

    current = this.intersectYzPlane(ray);
    last =  this.intersectYzPlane(old_ray);
    
    if(current.i2d.length() < 0.2 * this.children[0].getSize())
      return null;

    var last_angle = Math.atan2(last.i2d.y, last.i2d.x);
    var angle      = Math.atan2(current.i2d.y, current.i2d.x);

    var delta = angle - last_angle;
    var axis  = this.referenceOrientation.multiplyVector(this.controlOrientation.xAxis());
    
    var quat = new ros.math.Quaternion();
    quat.fromAngleAxis(delta, axis);

    this.rotationAngle += delta;
    return quat;

  },

  intersectYzPlane : function(ray)
  {
    var position = this.parent.getPosition(); 

    var xaxis = this.controlOrientation.xAxis();  
    var yaxis = this.controlOrientation.yAxis();  
    var zaxis = this.controlOrientation.zAxis();  

    var normal = this.referenceOrientation.multiplyVector(xaxis);
    var axis_1 = this.referenceOrientation.multiplyVector(yaxis); 
    var axis_2 = this.referenceOrientation.multiplyVector(zaxis); 

    var plane = new ros.geometry.Plane(position, normal);
    var origin_2d = new ros.math.Vector2(position.dotProduct(axis_1), position.dotProduct(axis_2));

    var intersect_3d;
    var intersect_2d;

    var dist = plane.intersectRay(ray);
    log('dist =  ' + dist);

    if(dist != null)
    {
      intersect_3d = ray.getPoint(dist);
      intersect_2d = new ros.math.Vector2(intersect_3d.dotProduct(axis_1), intersect_3d.dotProduct(axis_2));
      intersect_2d.subtract(origin_2d);
      return {i3d : intersect_3d, i2d: intersect_2d, distance : dist} ;  
    }

    ros_debug('null!!!!!!!!!!!!!!!!');

    return null;
  },

  getClosestPosOnAxis : function(ray)
  {
    var x_axis = this.controlOrientation.xAxis();
    var axis = this.referenceOrientation.multiplyVector(x_axis);

    var axis2 = (ray.getDirection()).crossProduct(axis); 
    var normal = axis2.crossProduct(ray.getDirection());

    var plane = new ros.geometry.Plane(ray.getOrigin(),normal);

    var axis_ray_origin = new ros.math.Vector3(this.parent.position.x,this.parent.position.y,this.parent.position.z);
    axis_ray_origin.subtract(new ros.math.Vector3(axis.x * 1000, axis.y * 1000, axis.z * 1000));
    
    var axis_ray = new ros.geometry.Ray(axis_ray_origin, axis);

    var dist = plane.intersectRay(axis_ray);
    return dist;
  },

  notify : function()
  {
    this.setCumMatrix();
  }

});


ros.visualization.InteractiveMarkers.InteractiveMarkerControl.ORIENTATION_MODE = {
                "INHERIT": 0, "FIXED": 1, "VIEW_FACING": 2};

ros.visualization.InteractiveMarkers.InteractiveMarkerControl.INTERACTION_MODE = {
                "NONE": 0, "MENU": 1, "BUTTON": 2, "MOVE_AXIS": 3, 
                "MOVE_PLANE": 4, "ROTATE_AXIS":5, "MOVE_ROTATE": 6};

