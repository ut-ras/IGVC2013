dojo.provide("scavenger.PointNNav");

dojo.require("scavenger.PointNNavCanvas");
dojo.require("dijit._Widget");
dojo.require("dijit._Templated");
dojo.require("dijit.form.Button");

dojo.declare("scavenger.PointNNav",[dijit._Widget, dijit._Templated], {
  
  templatePath : dojo.moduleUrl("scavenger", "templates/PointNNav.html"),
  button : {},
  pointEnable : false,
  navService : "/pointclick/navstart",
  headService : "/pointclick/headmove",
  cancelService : "/pointclick/cancel",
  navClick : false,
  directmoveto : false,

  postCreate : function() 
  {
    this.createCanvas();
    this.createButtons();
  },

  startup: function() {
    this.connect(this.canvasdijit,"mouseDown","mouseDown");
    this.connect(this.canvasdijit,"mouseUp","mouseUp");
    this.canvasdijit.startup();
  },

  createCanvas : function() {
    this.canvasdijit = new scavenger.PointNNavCanvas({},this.canvas);
  },

  createButtons : function() {
    this.button = {};
    this.button["NavigateTo"] = this.createButton("NavigateTo");
    this.button["DirectlyMoveTo"] = this.createButton("DirectlyMoveTo");
    this.button["Head"] = this.createButton("Head");
    this.button["Cancel"] = this.createButton("CancelNavGoal");
  },

  createButton : function(name) {
    var button = new dijit.form.Button({label:name, style:"width:150px"});
    this.buttonAttach.appendChild(button.domNode);
    this.addNewLine(this.buttonAttach);
    this.connect(button,"onClick",name);
    return button;
  },

  addNewLine : function(attachPoint)
  {
    var br = document.createElement('br');
    attachPoint.appendChild(br);
  },

  NavigateTo : function(event) {
    this.navClick = true;
    this.pointEnable = false;
    this.button["Head"].setAttribute('label',"Head : " + this.pointEnable);
  },

  DirectlyMoveTo : function(event) {
    this.directmoveto = true;
    this.pointEnable = false;
    this.button["Head"].setAttribute('label',"Head : " + this.pointEnable);
  },

  Head : function(event) {
    this.pointEnable = !(this.pointEnable);
    this.button["Head"].setAttribute('label',"Head : " + this.pointEnable);
    if(this.pointEnable) {
      this.navClick = false;
      this.directmoveto = false;
    }
  },

  CancelNavGoal: function(event) {

    topic = {};
    topic.target = this.canvasdijit.fixed_frame;
    topic.near = {};
    topic.near.header = {};
    topic.near.header.frame_id = "";
    topic.near.point = {};
    topic.near.point.x = 0; 
    topic.near.point.y = 0; 
    topic.near.point.z = 0; 

    topic.far = {};
    topic.far.header = {};
    topic.far.header.frame_id = "";
    topic.far.point = {};
    topic.far.point.x = 0
    topic.far.point.y = 0;
    topic.far.point.z = 0;
    //topic.use_navigation = false;

    console.log("Canceling goal");
    ros.callService(this.cancelService,topic,this.nop);             
  },

  mouseDown : function(gl,button, x,y) {
    console.log("X",x,"Y",y);
    var topic = {};
    var ray = this.canvasdijit.getWorldRay(x,y);
    var root = this.canvasdijit.tf.tree.getRootNode();
    var world_frame = root.frame_id;
    console.log(world_frame);
    topic = {};
    topic.target = this.canvasdijit.fixed_frame;
    topic.near = {};
    topic.near.header = {};
    topic.near.header.frame_id = world_frame;
    topic.near.point = {};
    topic.near.point.x = ray.near[0]; 
    topic.near.point.y = ray.near[1]; 
    topic.near.point.z = ray.near[2]; 

    topic.far = {};
    topic.far.header = {};
    topic.far.header.frame_id = world_frame;
    topic.far.point = {};
    topic.far.point.x = ray.far[0];
    topic.far.point.y = ray.far[1];
    topic.far.point.z = ray.far[2];
    topic.use_navigation = true;

    if(this.navClick) {
      topic.use_navigation = true;
      ros.callService(this.navService,topic,this.nop);
      this.navClick = false;
    }
    else if(this.directmoveto) {
      topic.use_navigation = false;
      ros.callService(this.navService,topic,this.nop);
      this.directmoveto = false;
    }
    else if(this.pointEnable)
      ros.callService(this.headService,topic,this.nop);
    else {
      console.log("Nothing");
    }
  },

  mouseUp : function(gl,button, x,y) {
//    console.log("x",x,"y",y);
  },

  nop : function() { },

});
