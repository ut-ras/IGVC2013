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

/* modified Date 06.2011 */

ros.visualization.SceneViewer = Class.extend({
    init: function()
    {
	this.xform = null;
	this.camera = null;
	this.interactor = null;
	this.interactor2 = null;
	this.nodeMap = new ros.Map();
	this.tf = null;
	this.currentNodeId = 0;
	this.pickmanager = new ros.visualization.PickManager(this);
	this.fixed_frame = "/base_link";
	this.clear_color =[0.0,0.0,0.0,0.0];

	// callbacks
	this.onpick = null;
	this.ondraw = null;

	this.pickedNode = null;

	// interactors
	this.orbitInteractor = null;
	this.puppetInteractor = null;

	// listenerList
	this.listenerList = [];

	this.screenLock = false;
    },

    addNode : function(node) {
	var that = this;
	
	var nodeId = this.currentNodeId;
	this.currentNodeId++;
	
	function onloadNode(node) {
	    var frame_id = node.frame_id;
	    var innernode = that.nodeMap.find(frame_id);

	    if(innernode == null)
		innernode = new ros.Map();
	    
	    innernode.insert(nodeId,node);
	    that.nodeMap.insert(frame_id,innernode);
	    that.ui.requestDraw();

	    ros_debug("Added a new inner node in the map:");
	    console.log(innernode);
	}	

	node.load(onloadNode, this);
	return nodeId;
    },

    addNodeWithoutLoading : function(node) {
	var frame_id = node.frame_id;
	var nodeId = this.currentNodeId;
	this.currentNodeId++;

	var innernode = this.nodeMap.find(frame_id);
	if(innernode == null)
	    innernode = new ros.Map();
	
	innernode.insert(nodeId,node);
	this.nodeMap.insert(frame_id, innernode);
	this.ui.requestDraw();
    },

    removeNode: function(nodeId) {
	ros_debug("scene viewer is trying to remove:");
	console.log(nodeId);
	var nodes = this.nodeMap.valSet();
	console.log(nodes);
	for(var n in nodes)
	{
	    // FIX-ME!!!
	    // For some reason the nodeId returned by addNode method
	    // is one greater than the nodeId that the scene viewer keeps
	    // That's why we can never delete the first visualization node
	    // we add, UNLESS we manipulate the nodeId here by substracting 1.
	    var node = nodes[n].find(nodeId-1);
	    console.log(node);
	    if(node) {
		//      node.remove();
		nodes[n].remove(nodeId-1);
		
	    }
	}
	
	//console.log(this.ui);
	this.ui.requestDraw();
    },

    getNode: function(nodeId){
	var nodes = this.nodeMap.valSet();
	for(var n in nodes){
	    var node = nodes[n].find(nodeId-1);
	    if(node) {
		return node;
	    }
	}
	
    },

    load : function(gl) {
	this.xform = new SglTransformStack();
	this.camera = new ros.visualization.PerspectiveCamera();
	this.orbitInteractor = new ros.visualization.OrbitInteractor(this.camera);
	this.puppetInteractor = new ros.visualization.PuppetInteractor(this.camera,this);
	this.selectInteractor = new ros.visualization.SelectInteractor(this.camera,this);
	this.interactor = this.orbitInteractor;
	this.interactor2 = this.selectInteractor;
	this.interactor2.interactorMatrix = this.orbitInteractor.interactorMatrix;
	this.enableControls(true);
    },

    keyDown: function(gl, keyCode, keyString) {
	if(keyString == "O") {
	    this.puppetInteractor.reset();
	    this.interactor2 = this.selectInteractor;
	    this.interactor2.interactorMatrix = this.orbitInteractor.interactorMatrix;
	    log('Orbit Interactor');
	}
	else if(keyString == "S") {
	    this.interactor2 = this.selectInteractor;
	    this.interactor2.interactorMatrix = this.orbitInteractor.interactorMatrix;
	    this.enableControls(true);
	    log('Select Interactor');
	}/*
	   else if(keyString == "I")
	   {
	   log('Bounding box enable');
	   var nodelist = this.nodeMap.valSet();

	   for(var i in nodelist) {
           var nodes = nodelist[i].valSet();

           for(var j in nodes) {
           nodes[j].setBoundingBoxEnable(true);
           }
	   }
	   
	   }
	   else if(keyString == "U")
	   {
	   log('Bounding box disable');
	   var nodelist = this.nodeMap.valSet();

	   for(var i in nodelist) {
           var nodes = nodelist[i].valSet();

           for(var j in nodes) {
           nodes[j].setBoundingBoxEnable(false);
           }
	   }
	   }
	   else if(keyString == "L")
	   {
	   this.screenLock = this.screenLock == true?false:true;

	   if(this.screenLock)
           log('Lock screen');
	   else 
           log('Unlock screen');

	   }*/
	this.interactor.keyDown(gl, keyCode, keyString);
	this.notifyListener();
    },

    keyUp: function(gl, keyCode, keyString)
    {
	this.notifyListener();
    },

    resize : function(gl, width, height) {
	this.camera.resize(gl, width, height);
	this.interactor.resize(gl, width, height);
    },
    
    update : function(gl, dt) {
	;
    },

    drawScene : function(gl, xform) 
    {
	var frameList = this.nodeMap.keySet();
	var nodesList = this.nodeMap.valSet();

	for(var f in frameList)
	{
	    var frame_id = frameList[f];
	    var nodes = nodesList[f]
	    var matrix = this.lookupTransform(frame_id);
	    
	    if(matrix == null) 
		continue;
	    xform.model.push();
	    //      xform.model.multiply(matrix);
	    this.drawNodes(gl, xform,nodes);
	    xform.model.pop();
	}
    },

    drawNodes : function(gl,xform, nodelist)
    {
	var nodes = nodelist.valSet();

	for(var n in nodes)
	{
	    this.drawNode(gl, xform, nodes[n]);
	}
    },

    drawNode : function(gl, xform, node)
    {
	var matrix = this.lookupTransform(node.frame_id);

	if(matrix == null || !(node.enabled))
	    return;

	xform.model.push();
	xform.model.multiply(matrix);

	if(node.children.length == 0) {
	    xform.model.push();
	    xform.model.multiply(node.matrix);

	    node.draw(gl, xform);
	    xform.model.pop();
	}
	else {
	    for(var c in node.children)
	    {
		this.drawNode(gl, xform, node.children[c]);
	    }
	}
	xform.model.pop();
    },
    
    lookupTransform : function(frame_id)
    {
	if(frame_id =="" || frame_id == "/")
	    return sglIdentityM4();
	else 
	    return this.tf.transformLookup.find(frame_id);
    },

    getFixedFrameTransform : function(fixed_frame)
    {
	if(!this.tf.tree)
	    return;

	var root_node = this.tf.tree.getRootNode();

	if(!root_node)
	    return;

	var world_frame = root_node.frame_id;
	var matrix = this.tf.lookupTransformMartix(world_frame, fixed_frame);
	return matrix;
    },

    draw : function(gl)
    {
	var w = this.ui.width;
	var h = this.ui.height;

	// TODO: this is a hack, it would be here if the resize event callback would work
	// camera resizing was making overlay wrong.
	this.resize(gl,w,h);


	if(this.nodeMap.size() == 0)
	    return;

	if(!this.tf)
	    return;

	// clear buffers
	gl.clearColor(this.clear_color[0], this.clear_color[1], this.clear_color[2], this.clear_color[3]);
	gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
	gl.enable(gl.DEPTH_TEST);
	gl.depthFunc(gl.LESS);
	gl.disable(gl.CULL_FACE);

	// set up view port
	gl.viewport(0, 0, w, h);

	// setup projection matrix
	this.xform.projection.loadIdentity();
	this.xform.projection.multiply(this.camera.projectionMatrix);

	// setup view matrx
	this.xform.view.loadIdentity();

	var viewmatrix = this.getViewMatrix();
	this.xform.view.multiply(viewmatrix);

	// setup model matrix
	this.xform.model.loadIdentity();
	this.drawScene(gl, this.xform);

	// callback
	if(this.ondraw) {
	    this.ondraw(gl, this.xform);
	}

	this.xform.model.pop();

	gl.disable(gl.DEPTH_TEST);
	gl.disable(gl.CULL_FACE);
    },

    getViewMatrix : function()
    {
	var matrix = sglIdentityM4();
	matrix = sglMulM4(matrix, this.camera.viewMatrix);
	matrix = sglMulM4(matrix, this.interactor.interactorMatrix);

	var fixed_frame_matrix = this.getFixedFrameTransform(this.fixed_frame);
	if(fixed_frame_matrix) {
	    matrix = sglMulM4(matrix,fixed_frame_matrix);
	}

	return matrix;
    },

    enableControls: function(enable)
    {
	var nodeMap = this.nodeMap.valSet();

	for(var n in nodeMap)
	{
	    var nodes = nodeMap[n].valSet();

	    for(var i in nodes)
	    {
		var node = nodes[i];

		this.enableChildren(node, enable);
	    }
	}
    },

    enableChildren : function(node,enable)
    {
	if(node.interaction_mode > 0)
	{
	    node.setEnable(enable);
	    node.setPickable(enable);
	}
	if(node.interaction_mode == 0)
	{
	    node.setEnable(!enable);
	    node.setPickable(!enable);
	}

	for(var c in node.children)
	{
	    this.enableChildren(node.children[c],enable);
	}
    },


    unSelectAll : function()
    {
	var nodeMapList = this.nodeMap.valSet();

	for(var i in nodeMapList)
	{
	    var nodes = nodeMapList[i].valSet();

	    for(var j in nodes)
	    {
		var node = nodes[j];

		node.setBoundingBoxEnable(false);
	    }
	}
    },

    pick: function(gl, mouseButtonsDown, x, y)
    {
	var w = this.ui.width;
	var h = this.ui.height;

	// TODO: this is a hack, it would be here if the resize event callback would work
        this.camera.resize(gl,w,h);

	// remove previous selection
	//    this.unSelectAll();

	var worldRay = this.getWorldRay(x, y);

	// find object intersecting the ray
	var pickInfo = this.pickmanager.pick(worldRay, this.xform);

	// enable bounding box model in scene graph this can be overwritten by the callback
	if(pickInfo.valid) {
	    //      pickInfo.closestNode.setBoundingBoxEnable(true);
	    this.pickedInfo = pickInfo;
	    //      log(this.pickedNode.frame_id);
	}
	else {
	    var menu = {};
	    menu.attributes = "";
	    menu.items = [];
	    RightContext.setMenu(menu);
	}

	this.ui.requestDraw();

	// callback
	if(this.onpick) {
	    this.onpick(pickInfo);
	}

	return pickInfo;
    },

    getWorldRay: function(x, y)
    {
	var w = this.ui.width;
	var h = this.ui.height;

	// setup projection matrix                                                          
	this.xform.projection.loadIdentity();
	this.xform.projection.multiply(this.camera.projectionMatrix);
	
	// setup view matrix
	this.xform.view.loadIdentity();
	this.xform.view.multiply(this.camera.viewMatrix);
	this.xform.view.multiply(this.interactor.interactorMatrix);
	
	var fixed_frame_matrix = this.getFixedFrameTransform(this.fixed_frame);
	
	if(fixed_frame_matrix) {
	    this.xform.view.multiply(fixed_frame_matrix);
	}
	
	// setup model matrix
	this.xform.model.loadIdentity();
	
	// create a ray in world coordinates
	return this.pickmanager.clientPositionToWorldRay(x, y, this.xform, w, h);
    },

    mouseDown: function(gl, button, x, y) {
	this.pickedNode = this.pick(gl, button, x, y);

	if(this.pickedNode.valid)
	    this.interactor2.mouseDown(gl,this.ui.mouseButtonsDown,x,y,this.pickedNode);
	else if(!this.screenLock) {
            this.interactor.mouseDown(gl,this.ui.mouseButtonsDown,x,y);
            this.notifyListener();
	}
    },

    mouseUp: function(gl, button, x, y) {
	if(this.pickedNode && this.pickedNode.valid) {
	    this.interactor2.mouseUp(gl,x, y, button, this.ui.keysDown);
	    this.pickedNode = null;
	}
	else if(!this.screenLock) {
	    this.interactor.mouseUp(gl,x, y, button, this.ui.keysDown);
	    this.notifyListener();
	}

    },

    mouseMove : function(gl, x, y) {
	if(this.pickedNode && this.pickedNode.valid) {
	    this.interactor2.mouseMove(gl, x, y, this.ui.mouseButtonsDown, this.ui.keysDown);
	}
	else if(!this.screenLock){
	    this.interactor.mouseMove(gl, x, y, this.ui.mouseButtonsDown, this.ui.keysDown);

	    if(this.ui.mouseButtonsDown[0])
		this.notifyListener();
	}

    },

    mouseWheel : function(gl, wheelDelta, x, y) {
	if(this.pickedNode && this.pickedNode.valid)
	    this.interactor2.mouseWheel(gl, wheelDelta, x, y, this.ui.mouseButtonsDown, this.ui.keysDown);
	else if(!this.screenLock) {
	    this.interactor.mouseWheel(gl, wheelDelta, x, y, this.ui.mouseButtonsDown, this.ui.keysDown);
	    this.notifyListener();
	}
    },

    setOnPick: function(func) {
	this.onpick = func;
    },
    
    setOnDraw: function(func) {
	this.ondraw = func;
    }, 

    addListener : function(l)
    {
	this.listenerList.push(l);
    },

    notifyListener : function()
    {
	for(var l in this.listenerList)
	{
	    this.listenerList[l].notify();
	}
    },

    receivedInteractiveMarker : function()
    {
	if(this.interactor2 == this.selectInteractor)
	    this.enableControls(true);
    },
});


