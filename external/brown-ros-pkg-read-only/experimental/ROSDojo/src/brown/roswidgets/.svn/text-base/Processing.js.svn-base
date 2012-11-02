dojo.provide("roswidgets.Processing");

dojo.require("roswidgets.Utils");

dojo.require("dijit._Widget");

dojo.declare("roswidgets.Processing",[dijit._Widget], {
  source : "lib/processingjs/empty.pde", 
  module : "roswidgets",
  width: 320,
  height : 240,

  postCreate : function() 
  {
    this.createCanvas();
  },

  startup: function()
  {
  },

  createCanvas : function() {
    this.canvas = document.createElement('canvas'); 
    this.canvas.width = this.width +"px";
    this.canvas.height = this.height +"px";
    this.canvas.setAttribute("style","border: 2px solid black");

    var source = roswidgets.Utils.loadSource(this.source,this.module);
    this.pIns = new Processing(this.canvas,source);
    this.domNode.appendChild(this.canvas);
  },

  getProcessingIns : function() {
    return this.pIns;
  },
});
