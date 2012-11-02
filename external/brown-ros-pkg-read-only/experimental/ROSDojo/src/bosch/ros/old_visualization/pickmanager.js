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
 * A pick info object
 * @constructor
 */
ros.visualization.PickInfo = Class.extend({
  init: function() 
  {
    this.valid = false;
    this.closestNode = null;
    this.closestDistance = null;
    this.worldIntersectionPosition = null;
    this.linkarray = null;
  },
  
});  

ros.visualization.PickManager = Class.extend({
  init: function(sceneviewer) {
    this.sceneviewer = sceneviewer;
  },

  /**
   * Convert a pixel position relative to the top left corner of the client area
   * into the corresponding ray through the frustum in world space.
   */
  clientPositionToWorldRay: function(clientXPosition,
                                     clientYPosition,
                                     xform,
                                     clientWidth,
                                     clientHeight) {
    
    var viewport = sglV4([0,0,clientWidth,clientHeight]);
    var projectionMatrix = xform.projectionMatrix;
    var modelViewMatrix = xform.modelViewMatrix;
    
    // Apply inverse view-projection matrix to get the ray in world coordinates.
    return {
        near:ros.visualization.unproject(clientXPosition, clientYPosition, 0, modelViewMatrix, projectionMatrix, viewport),
        far: ros.visualization.unproject(clientXPosition, clientYPosition, 1, modelViewMatrix, projectionMatrix, viewport)
    };
  },
  
  intersectNodeWithRay: function(worldRay,xform,node) {
    var inverseWorldMatrix = xform.modelMatrixInverse;
    var relativeNear = ros.visualization.transformPoint(inverseWorldMatrix, worldRay.near);
    var relativeFar = ros.visualization.transformPoint(inverseWorldMatrix, worldRay.far);

//    return node.model.bounding_box.intersectRay(relativeNear, relativeFar);
    var rayInfo ={};
    if(node.model == null && node.model.bounding_box == null) {
      rayInfo = {};
      rayInfo.intersected = false;
      return rayInfo;
    }
    else {
      rayInfo = node.model.bounding_box.intersectRay(relativeNear, relativeFar);
    }

    if(rayInfo.intersected)
    {
      var element = node.model; 

      rayInfo.intersected = false;
      if(element == null) {
        if(node.vertices != null) {
          return node.intersectRay(relativeNear,relativeFar);
        }

        return rayInfo;
      }
      else 
        return element.intersectRay(relativeNear,relativeFar);
    }
    return rayInfo;
  },
  
  pick: function(worldRay, xform)
  {
    var pickinfo = new ros.visualization.PickInfo();
    var linkarray = new Array();

    var frameList = this.sceneviewer.nodeMap.keySet();
    var nodesList = this.sceneviewer.nodeMap.valSet();

    for(var f in frameList)
    {
      var frame_id = frameList[f];
      var nodes = nodesList[f];
      var matrix = this.sceneviewer.lookupTransform(frame_id);

      if(matrix == null)
        continue;

      xform.model.push();
//      xform.model.multiply(matrix);
      this.pickNodes(worldRay, xform, pickinfo, nodes,linkarray);
      xform.model.pop();
    }
    return pickinfo;
  },

  pickNodes : function(worldRay, xform, pickinfo, nodelist,linkarray)
  {
    var nodes = nodelist.valSet();

    for(var n in nodes)
    {
      var node = nodes[n];

      linkarray.push(node);
      this.pickNode(worldRay, xform, pickinfo,node, linkarray);
      linkarray.pop();
    }
  },

  pickNode : function(worldRay, xform, pickinfo, node, linkarray)
  {
    var matrix = this.sceneviewer.lookupTransform(node.frame_id);

    if(matrix == null || !node.isPickable())
      return;

    xform.model.push();
    xform.model.multiply(matrix);



    if(node.children.length == 0 && node.model)
    {
      xform.model.push();
      xform.model.multiply(node.matrix);
//      xform.model.scale(node.scale[0],node.scale[1], node.scale[2]);

      var intersectionInfo = this.intersectNodeWithRay(worldRay, xform, node); 

      if(intersectionInfo.intersected) 
      {
        var worldMatrix = xform.modelMatrix;
        var worldIntersectionPosition = ros.visualization.transformPoint(worldMatrix, intersectionInfo.position);
        var distance = sglSqLengthV3(sglSubV3(worldRay.near, worldIntersectionPosition));

        if(pickinfo.closestNode)
        {
          if(distance < pickinfo.closestDistance) {
            pickinfo.valid = true;
            pickinfo.closestDistance = distance;
            pickinfo.closestNode = node;
            pickinfo.worldIntersectionPosition = worldIntersectionPosition;
            pickinfo.linkarray = new Array();

            for(var l in linkarray)
              pickinfo.linkarray.push(linkarray[l]);
          }
        }
        else 
        {
          pickinfo.valid = true;
          pickinfo.closestDistance = distance;
          pickinfo.closestNode = node;
          pickinfo.worldIntersectionPosition = worldIntersectionPosition;
          pickinfo.linkarray = new Array();

          for(var l in linkarray)
            pickinfo.linkarray.push(linkarray[l]);
        }
      }
      xform.model.pop();
    }
    else {
      for(var c in node.children)
      {
        var child = node.children[c];
        linkarray.push(child);
        this.pickNode(worldRay, xform, pickinfo,child,linkarray);
        linkarray.pop();
      }
    }
    
    xform.model.pop();
  },
  
});
