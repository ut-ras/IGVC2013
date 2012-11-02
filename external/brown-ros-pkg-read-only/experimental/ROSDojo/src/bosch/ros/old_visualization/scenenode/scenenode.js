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

ros.visualization.SceneNode = Class.extend({
    init: function(vm)
    {
	this.vm = vm;
	this.frame_id = "";
	this.position = new ros.math.Vector3();
	this.orientation = new ros.math.Quaternion();
	this.matrix = sglIdentityM4();
	this.scale = [1.0,1.0,1.0];

	// Scene node can have either children or model
	// only leaf node is supposed to have model
	this.children = [];
	this.model = null;

	this.enabled = true;
	this.pickable = false;
	this.bounding_box = null;
	this.bounding_box_enable = false;

	this.parent = null;

	this.lastRotate = new ros.math.Quaternion(1,0,0,0);
	this.lastTranslate = new ros.math.Vector3(0,0,0);
	
	this.nodeId = null;
	
    },

    calculateMatrixFromPose: function()
    {
	var matrix = sglIdentityM4();
	var translation = sglTranslationM4V([this.position.x, this.position.y, this.position.z]);
	var rotation = sglGetQuatRotationM4([this.orientation.x, this.orientation.y, this.orientation.z, this.orientation.w]);
	var scale  = sglScalingM4V(this.scale);
	//console.log("CALCULATE MATRIX FROM POSE");
	matrix = sglMulM4(matrix, translation);
	matrix = sglMulM4(matrix, rotation);
	matrix = sglMulM4(matrix, scale); 

	this.matrix = matrix;
    },

    setVM : function(vm)
    {
	this.vm = vm;

	for(var i in this.children)
	{
	    this.children[i].setVM(vm);
	}
    },

    setPose: function(pose)
    {
	var position = pose.position;
	var orientation = pose.orientation;
	if(orientation.x == 0 && orientation.y ==0 && orientation.z == 0  && orientation.w ==0)
	    orientation.w = 1;

	this.position = new ros.math.Vector3(position.x, position.y, position.z);
	this.orientation = new ros.math.Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
	this.orientation.normalise();
	this.setCumMatrix();
    },

    setPosition: function(position)
    {
	this.position = new ros.math.Vector3(position.x, position.y, position.z);
	this.setCumMatrix();
    },

    getPosition : function()
    {
	return new ros.math.Vector3(this.position.x,this.position.y,this.position.z);
    },

    setOrientation: function(orientation)
    {
	if(orientation.x == 0 && orientation.y ==0 && orientation.z == 0  && orientation.w ==0)
	    orientation.w = 1;
	this.orientation = new ros.math.Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
	
	this.setCumMatrix();
    },

    getOrientation: function()
    {
	var q = new ros.math.Quaternion(this.orientation.w, this.orientation.x, this.orientation.y, this.orientation.z);
	return q;
    },

    setFrame: function(frame_id)
    { 
	ros_debug("SceneNode says: frame_id is "+frame_id);
	//console.log("My parent is");
	//console.log(this.parent);
	//console.log("My nodeId is");
	//console.log(this.nodeId);
	//console.log("My vm is");
	//console.log(this.vm);
	//console.log("My children are");
	//console.log(this.children);
	//console.log("My model is");
	//console.log(this.model);
	
	this.frame_id = ros.tf.formatFrameID(frame_id);
    },

    setScale: function(scale)
    {
        //console.log(" My Scale is:");
	//console.log(scale);
	this.scale = scale;
    },

    setCumMatrix : function()
    {
	if(this.parent)
	{

	    //console.log("SET CUM. MATRIX PARENT TRUE");
	    var matrix = sglIdentityM4();
	    var translation = sglTranslationM4V([this.position.x, this.position.y, this.position.z]);
	    var rotation = sglGetQuatRotationM4([this.orientation.x, this.orientation.y, this.orientation.z, this.orientation.w]);
	    var scale  = sglScalingM4V(this.scale);
	    matrix = sglMulM4(matrix, translation);
	    matrix = sglMulM4(matrix, rotation);
	    matrix = sglMulM4(matrix, scale);

	    var pmatrix = sglIdentityM4();
	    var pscale  = sglScalingM4V(this.parent.scale);
	    pmatrix = sglMulM4(pmatrix, this.parent.matrix);
	    pmatrix = sglMulM4(pmatrix, pscale);

	    this.matrix = sglMulM4(pmatrix, matrix);
	}
	else {
	    this.calculateMatrixFromPose();
	}

	for(var c in this.children)
	{
	    this.children[c].setCumMatrix();
	}
    },

    isPickable: function()
    {
	return this.pickable;
    },

    setPickable: function(pickable)
    {
	this.pickable = pickable;

	for(var i in this.children)
	{
	    this.children[i].setPickable(pickable);
	}
    },

    setModel: function(model)
    {
	this.model = model;
	this.model.parent = this;

    }, 

    setHighlightPass: function(pass)
    {
	if(this.model)
	    this.model.setHighlightPass(pass)
	else 
	{
	    for(var c in this.children)
	    {
		this.children[c].setHighlightPass(pass);
	    }
	}
    },

    setAlpha: function(alpha)
    {
	if(this.model)
	    this.model.setAlpha(alpha)
	else 
	{
	    for(var c in this.children)
	    {
		this.children[c].setAlpha(alpha);
	    }
	}
    },



    addChildNode: function(child)
    {
	child.parent = this;
	this.children.push(child);
    },

    removeChildNode: function(child)
    {
	// maybe added later

    },

    setEnable: function(enabled)
    {
	this.enabled = enabled;
    },

    setBoundingBoxEnable: function(enabled)
    {
	this.bounding_box_enable = enabled;
	for(var c in this.children)
	{
	    this.children[c].setBoundingBoxEnable(enabled);
	}
    },

    setParent : function(p)
    {
	this.parent = p;
    },

    translate : function(delta)
    {
	this.lastTranslate = delta;
	this.position.add(delta);
	this.setCumMatrix();
    },

    rotate : function(delta_quat)
    {
	this.lastRotate = delta_quat;
	this.orientation.multiplyQuat(delta_quat);
	
	this.setCumMatrix();
    },

    intersectRay: function(start, end)
    {
	var r;
	for(var c in this.children)
	{
	    r = this.children[c].intersectRay(start,end);      
	    if(r.intersected)
		return r;
	}

	if(this.model)
	{
	    r = this.model.intersectRay(start,end);

	    return r;
	}

	// if the node has neither model nor children, it has problem.
	return null;
    },

    load: function(callback)
    {
	if(this.model) {
	    this.model.load();
	}
	else {
	    for(var c in this.children)
	    {
		this.children[c].load();
	    }
	}
	
	var async = (callback) ? (true) : (false);
	if (async) {
	    callback(this);
	}
    },

    draw: function(gl, xform)
    {
	if(this.model)
	{
	    this.model.draw(gl,xform);

	    if(this.bounding_box_enable) {
		this.model.bounding_box.draw(gl,xform);
	    }
	}
    },

});

ros.include('visualization/scenenode/laserscannode');
ros.include('visualization/scenenode/gridnode');
ros.include('visualization/scenenode/pointcloud2node');
ros.include('visualization/scenenode/coordinateframenode');
ros.include('visualization/scenenode/tfnode');
ros.include('visualization/scenenode/robotnode');
ros.include('visualization/scenenode/mapnode');

