dojo.provide("userstudy.ImageSegmentation");

dojo.require("userstudy.Utils");
dojo.require("roswidgets.MJPEGViewer2");

dojo.require("dijit.form.Button");
dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.declare("userstudy.ImageSegmentation", [dijit._Widget, dijit._Templated], {

  templateString : dojo.cache("userstudy", "templates/ImageSegmentation.html"),
  videoTopic : "/remote_lab_cam2/image_raw?invert",
  videoUrl : "localhost",
  videoWidth : 250,
  videoHeight : 200, 
  segService : "/imageseg/segment",
  pickupService : "/imageseg/pickup",
  resetService : "/imageseg/reset",
  flag : false,

  postCreate : function() {
    dojo.addClass(this.domNode,"image-segmentation");
    this.utils = new userstudy.Utils();
    this.createLeft();
    this.createRight();
    this.createLog();
  },

  createLeft : function() {
    this.createCanvas();
  },

  createRight : function() {
    var video = new roswidgets.MJPEGViewer2({topic:this.videoTopic, url:this.videoUrl, width:this.videoWidth,height:this.videoHeight},this.videoAttach);
    this.createButtons();
  },

  createButtons : function() {
    this.button = {};
    this.button["Seg"] = this.createButton("Segmentation");
    this.button["Pickup"] = this.createButton("Pickup");
    this.button["Reset"] = this.createButton("Reset");
  },

  createButton : function(name) {
    var button = new dijit.form.Button({label:name, style:"width:150px"});
    this.buttonsAttach.appendChild(button.domNode);
    this.addNewLine(this.buttonsAttach);
    this.connect(button,"onClick",name);
    return button;
  },

  addNewLine : function(attachPoint)
  {
    var br = document.createElement('br');
    attachPoint.appendChild(br);
  },

  Segmentation : function(event) {
    var sx = this.pIns.getStartPosX();
    var sy = this.pIns.getStartPosY();
    var ex = this.pIns.getEndPosX();
    var ey = this.pIns.getEndPosY();

    var st_x = (sx > ex)? ex:sx;
    var fi_x = (sx < ex)? ex:sx;
    var st_y = (sy > ey)? ey:sy;
    var fi_y = (sy < ey)? ey:sy;

    var req = {};
    req.topleft = { x:st_x,y:st_y,z:0};
    req.bottomright = { x:fi_x,y:fi_y,z:0};

    var that = this;
    that.log('Hello');
    console.log('Hello');
    ros.callService(this.segService,dojo.toJson([req]),function(msg) {
      that.pIns.resetImage(resp.img);
      that.flag = true;
      that.log('Received Segmented Image');
    });
  },

  Pickup : function(event) {
    if(this.flag == false) {
      this.log('Fail : Segment the image first.');
      return;
    }

    var that = this;
    ros.callService(this.pickupService,dojo.toJson([]),function(resp) {
        if(resp.success == false)
        {
          that.log(resp.msg);
        }
        else {
          that.log('Success!');
        }
        that.resetClick();
      });
    that.log('Reset Image');
  },

  Reset : function(event) {
    var that = this;
    ros.callService(this.resetService,dojo.Json([]),function(resp) {
        that.log('Received Image');
        that.pIns.resetImage(resp.img);
        that.flag = false;
      });
    that.log('Reset Image');
  },

  createLog : function() {
    var label =  document.createElement('label');
    label.innerHTML = "- Log -";
    this.logDiv = document.createElement('div');
    this.logAttach.appendChild(label);
    this.logAttach.appendChild(this.logDiv);
  },

  log : function(msg) {
    this.logDiv.innerHTML += msg + '<br/>';
    //this.logDiv.attr({scrollTop : this.logDiv.attr('scrollHeight')});
  },

  createCanvas : function() {
    this.canvas = document.createElement('canvas'); 
    this.canvas.width = "50px";
    this.canvas.height = "300px";
    this.canvas.setAttribute("style","border: 2px solid black");

    var source = userstudy.Utils.loadSource("lib/image_seg.pde");
    this.pIns = new Processing(this.canvas,source);
    this.canvasAttach.appendChild(this.canvas);
  },
    
});
