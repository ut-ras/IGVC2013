dojo.provide("bosch.VisualizationUI");

dojo.require("bosch.Visualization");
dojo.require("bosch.VisualizationControl");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.declare("bosch.VisualizationUI",[dijit._Widget,dijit._Templated], {

  templateString : dojo.cache("bosch", "templates/VisualizationUI.html"),

  postCreate : function() {
    dojo.addClass(this.domNode,"middlepanel");
    // control panel
    this.controlPanel = new bosch.VisualizationControl({},this.controlAttach);

    // webgl canvas
    this.visualization = new bosch.Visualization({},this.visualizationAttach);
  },

  startup : function() {
    this.visualization.startup();
    this.controlPanel.startup(this.visualization.vm,this.visualization.nodeHandle);
  },
});
