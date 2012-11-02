dojo.provide("bosch.VisualizationControl");

dojo.require("roswidgets.MJPEGViewerWrapper");
dojo.require("bosch.Utils");

dojo.require("dojox.grid.DataGrid");
dojo.require("dijit.form.Button");
dojo.require("dijit.form.DropDownButton");
dojo.require("dijit.Tooltip");
dojo.require("dijit.TooltipDialog");
dojo.require("dijit.Dialog");
dojo.require("dijit.form.TextBox");
dojo.require("dijit.form.FilteringSelect");
dojo.require("dojo.store.Memory");
dojo.require("dojo.data.ObjectStore");
dojo.require("dojo.data.ItemFileWriteStore");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.declare("bosch.VisualizationControl",[dijit._Widget, dijit._Templated],{
  
  vm : null,
  counter : 1,
  templatePath : dojo.moduleUrl("bosch", "templates/VisualizationControl.html"),
  
  postCreate: function() {
    dojo.addClass(this.domNode,"controlpanel");

    // mjpeg server
    this.video = new roswidgets.MJPEGViewerWrapper({},this.videoAttach);

    this.visHandler = new ros.visualization_widgets.VisualizationHandler();
    
    this.createDatagrid();
    this.createAvailSelector();
    dojo.connect(ros,"onConnecting",this,"onOpen");
  },

  startup : function(vm,node)
  {
    this.vm = vm;
    this.node = node;
    this.createGlobalOptionDialog();
    this.visHandler.onOpen(this.node,vm);

    this.grid.startup();
  },

  onOpen : function(url) {
    this.addButton.setAttribute('disabled',false);
    this.globalOptionButton.setAttribute('disabled',false);
  },

  createDatagrid : function() {
    var data = {
      identifier : 'name',
      items: []
    };

    var that = this;

    var layout = [ { name : "Name" , field : "name", editable : true},
                   { name : "Type" , field : "type", },
                   { name : "Property" , field : "button",   
                       type: dojox.grid.cells._Widget,
                       formatter: function() { 
                            var btn = new dijit.form.Button({label:"Property"}); 
                            that.connect(btn,"onClick","modifyProperty");
                            return btn;}
                   },
                   { name : "Remove" , field : "button",   
                       type: dojox.grid.cells._Widget,
                        formatter: function() { 
                            var btn = new dijit.form.Button({label:"Remove"}); 
                            that.connect(btn,"onClick","getDelete");
                            return btn;}
                   }
                 ];

    this.currentStore = new dojo.data.ItemFileWriteStore({data:data});
    

    this.grid = new dojox.grid.DataGrid({
        id : 'grid',
        store : this.currentStore,
        structure : layout,
        rowSelector:'20px'
        },this.layoutAttach);

    dojo.style(this.grid.domNode,"height","150px");
    dojo.style(this.grid.domNode,"width","95%");

    },
  
  createAvailSelector : function() {
      var availlist = this.visHandler.getAvailList();
      var availstore = new dojo.store.Memory({data:availlist});
  
      this.availSelect = new dijit.form.FilteringSelect({
        id:"availSelect",
        identifier : "type",
        placeHolder : "Select a node",
        store: availstore,
        autoComplete:true,
        searchAttr: "name"
        },this.availAttach);
  
      this.addButton = new dijit.form.Button({label:"Add",disabled:true},this.addAttach);
      this.connect(this.addButton,"onClick","addNode");
  
      this.globalOptionButton = new dijit.form.DropDownButton({label:"Global Option",disabled:true,
                                                                  dropDownPosition:["above","after"]},this.globalOptionAttach);
    },

  createGlobalOptionDialog : function()
  {
    var div = document.createElement('div');
    var dd = document.createElement('strong');
    var d2 = document.createElement('label');
    d2.innerHTML = "Fixed Frame : ";
    dd.appendChild(d2);
    div.appendChild(dd);
                                                                                
    this.fixed_frame_textbox = new dijit.form.TextBox({});
    this.fixed_frame_textbox.attr('value',this.vm.scene_viewer.fixed_frame);
    div.appendChild(this.fixed_frame_textbox.domNode);
    var br = document.createElement('br');
    div.appendChild(br);
                                                                                
    this.globalOptionSubmitButton = new dijit.form.Button({label:"Submit"});
    this.globalOptionSubmitButton.attr('type','submit');
    div.appendChild(this.globalOptionSubmitButton.domNode);
                                                                                
    this.connect(this.globalOptionSubmitButton,"onClick","submitGlobalOption");
                                                                                
                                                                                
    var dialog = new dijit.TooltipDialog({refreshOnShow : true});
    dialog.attr('content',div);
                                                                                
    this.globalOptionButton.attr('dropDown',dialog);
                                                                                
  },

  submitGlobalOption : function(e) 
  {
    this.vm.scene_viewer.fixed_frame = this.fixed_frame_textbox.value;
    this.fixed_frame_textbox.attr('value',this.vm.scene_viewer.fixed_frame);
                                                                             
    this.showTooltip("Updated");
  },

   // Shows a tooltip for 3 seconds with the message provided                                              
   showTooltip : function(label) {
       if (this.tooltipTimer) {
           window.clearTimeout(this.tooltipTimer);
       }
       dijit.showTooltip(label, this.globalOptionButton.domNode, [ "after", "above", "below", "before" ]);
       this.tooltipTimer = window.setTimeout(dojo.hitch(this, "hideTooltip"), 2000);
   },
 
   // Immediately hides any tooltip
   hideTooltip : function() {
       dijit.hideTooltip(this.globalOptionButton.domNode);
   },

  addNode : function() {
    console.log('add');
    var obj = dojo.clone(this.availSelect.item);
    console.log(obj);
    if(obj == undefined || obj == null)
      return;

    var that = this;

    this.currentStore.fetchItemByIdentity({'identity': obj.name, onItem : function(item)
        {
          if(item == null) {
            that.currentStore.newItem(obj);
          }
          else { 
            obj.name = obj.name + that.counter;
            that.currentStore.newItem(obj);
            that.counter = that.counter +1;
          }
          that.currentStore.save();

          that.visHandler.addNode(obj);

        }});
        

  },

  modifyProperty : function(event) {
    console.log('modify');
    var obj = this.grid.selection.getSelected()[0];
    if(obj == undefined || obj == null)
      return;

    var node = this.visHandler.getSceneNode(obj);
    this.createPropertyTooltip(node,obj.name);
  },

  createPropertyTooltip : function(node,name)
  {
    /*
     var div = document.createElement('div');                                        
     var dd = document.createElement('strong');
     var d2 = document.createElement('label');
     d2.innerHTML = "Topic : ";
     dd.appendChild(d2);
     div.appendChild(dd);
                                                                                 
     this.frame_textbox = new dijit.form.TextBox({});
     this.frame_textbox.attr('value',node.topic);
     div.appendChild(this.frame_textbox.domNode);
     var br = document.createElement('br');
     div.appendChild(br);
                                                                                 
     var propertySubmitButton = new dijit.form.Button({label:"Submit"});
     propertySubmitButton.attr('type','submit');
     div.appendChild(propertySubmitButton.domNode);
                                                                                 
     this.connect(propertySubmitButton,"onClick","submitProperty");
     */
     var dialog = new dijit.Dialog({refreshOnShow : true});
     this.property = new ros.visualization_widgets.PropertiesWidget(node);
     dialog.attr('content',this.property.getHtml(name));
     dojo.style(dialog.domNode,"width","300px");
     dojo.style(dialog.domNode,"height","400px");
     dojo.style(dialog.domNode,"background","white");

     var btn = dojo.byId(this.property.saveID);
     this.connect(btn,"onclick","submitProperty");

     var btn2 = dojo.byId(this.property.cancelID);
     this.connect(btn2,"onclick","cancelProperty");
     this.propertydialog = dialog;
    
     dialog.startup();
     dialog.show();
  },

  cancelProperty: function(event) {
    this.propertydialog.hide();
    this.propertydialog.destroy();
  },

  submitProperty : function(event) {
    this.property.onSave();
    /*
    var obj = this.grid.selection.getSelected()[0];
    var node = this.visHandler.getSceneNode(obj);
    this.vm.changeTopic(node,this.frame_textbox.value);
    */
 //   this.property.button.remove();

    var sn = this.property.sceneNode;

			sn.model.mesh.destroy();
			
			for(var k in sn.keys){
			    if(k == "current_frame"){
				sn.setFrame(sn.keys[k]);
			    }
			    else if(k == "topic"){
				if(sn.oldTopic != sn.topic){
				    sn.changeTopic(sn.topic);
				}
			    }
			    else{
				eval('sn.model.'+k+'=sn.'+k);
			    }
			}
			sn.model.load();

      this.cancelProperty(event);

  },

  getDelete : function(event) {
    var obj = this.grid.selection.getSelected()[0];
    var node = this.visHandler.getSceneNode(obj);
    var that = this;
    if(obj == undefined || obj == null)
      return;
    
    this.currentStore.fetchItemByIdentity({'identity': obj.name, onItem : function(item)
        {
          if(item == null) {}
          else { 
            console.log(item);
            // Unsubscribe from the topics if any
            for(var k in node.keys){
              if(k == "topic"){
                node.unsub();
              }   
            }   
            
            that.currentStore.deleteItem(item);
            that.currentStore.save();
            that.visHandler.removeNode(item);
          }
        }});
  }

});
