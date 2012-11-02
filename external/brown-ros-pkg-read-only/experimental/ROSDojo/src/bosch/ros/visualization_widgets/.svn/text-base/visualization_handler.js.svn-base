
ros.visualization_widgets.VisualizationHandler = Class.extend({
  init : function()
  {
    this.availiableNodeList = this.createAvailNodeList();
    this.currentNodeList = [];
    this.currentNodeHash = [];
//    this.globalOptions = []; TBA
  },

  getAvailList : function() { return this.availiableNodeList; },
  getCurrentNodeList : function() { return this.currentNodeList; },

  onOpen : function(node,vm)
  {
    this.vm = vm;
  },

  createAvailNodeList : function() 
  {
    var list = [ 
//          { name:"Axes",        type:"AxesNode"},
          { name:"Grid",        type:"GridNode"},
          { name:"LaserScan",   type:"LaserScanNode"},
          { name:"Map",         type:"MapNode"},
          { name:"PointCloud2", type:"PointCloudNode"}
              ];
    return list;
  },

  addNode : function(node) {
    var sn = this.vm.addVisualizationNode(node.type,node.name); // type, name
    console.log(sn);
    this.currentNodeList.push(node,sn.nodeId);
    this.currentNodeHash[node.name] = sn;
  },

  removeNode : function(node) {
    var id = this.currentNodeHash[node.name].nodeId;
    this.vm.removeVisualizationNode(id);
  },

  getSceneNode : function(node) {
    return this.currentNodeHash[node.name];
  },

});

