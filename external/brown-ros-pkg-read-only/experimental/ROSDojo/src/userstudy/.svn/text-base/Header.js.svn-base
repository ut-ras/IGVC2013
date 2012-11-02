dojo.provide("userstudy.Header");

dojo.require("dijit.form.Button");

dojo.require("dijit._Widget");
dojo.require("dijit.Dialog");

dojo.declare("userstudy.Header",dijit._Widget, {

  boardID : null,
  dialog : null,
  launchService : '/laucher',
  interfaceFile : dojo.cache("userstudy","json/interface.json"),

  postCreate : function() {
    this.interfaces = dojo.fromJson(this.interfaceFile);

    var button = new dijit.form.Button({label:"Open"});
    this.connect(button,"onClick","openDialog");
    this.domNode.appendChild(button.domNode);

    var dialog = new dijit.Dialog({refreshOnShow:true});
    dojo.style(dialog.domNode,"width","90%");
    dojo.style(dialog.domNode,"height","90%");
    dojo.style(dialog.domNode,"background","white");

    var content = this.createDialog();
    dialog.attr('title',"User Study");
    dialog.attr('content',content);
    
    this.dialog = dialog;

    var board = dojo.byId(this.boardID);

    dialog.startup();
    dialog.show();
    this.connectToServer();
  },

  connectToServer : function()
  {
    // connect to user study server
    // listens to status of experiment
    // based on status enable ready button. pop up the survery dialog
    // and end the experiment
  },

  openDialog : function(event) {
    console.log("Open");
    var content = this.createDialog();
    this.dialog.attr('content',content);
    this.dialog.show();
  },

  createDialog : function() {
    var html = document.createElement('div');
    for(i in this.interfaces.interfaces)
    {
      var btn = this.createButton(this.interfaces.interfaces[i].name);
      html.appendChild(btn.domNode);
    }
    this.lastdiv = document.createElement('div');
    html.appendChild(this.lastdiv);
    this.dialogHTML = html;

    return html;
  },

  createButton : function(name) {
    var button = new dijit.form.Button({label:name});
    this.connect(button,"onClick","selectInterface");

    return button;
  },

  selectInterface : function(event)
  {
    var selected = event.target.innerText; 

    for(i in this.interfaces.interfaces)
    {
      if(selected == this.interfaces.interfaces[i].name){ 
        break;
      }
    }
    selected = this.interfaces.interfaces[i];

    this.dialogHTML.removeChild(this.lastdiv);

    var div = document.createElement('div'); 
    
    var p = document.createElement('p');
    p.innerHTML = selected.name + " is selected";
    div.appendChild(p);
    //this.launchInterface(event.target.innerText);
    this.createTutorial(div,selected);
    this.createReadyButton(div,selected);
    this.dialogHTML.appendChild(div);
    this.lastdiv = div;

    this.prepareInterface(selected);
  },

  createTutorial : function(div,selected)
  {
    var iframe = document.createElement('iframe');
    iframe.setAttribute('src',selected.tutorialurl);
    iframe.setAttribute('width',420);
    iframe.setAttribute('height',315);
    //console.log(iframe);

    div.appendChild(iframe);
    var br = document.createElement('br');
    div.appendChild(br);
  },

  createReadyButton : function(div,selected)
  { 
    var btn = dijit.form.Button({label:"Ready",disabled:false});
    this.readyButton = btn;
    this.connect(this.readyButton,"onClick","closeDialog");
    div.appendChild(this.readyButton.domNode);
  },

  closeDialog : function(event) {
    this.dialog.hide();
  },

  prepareInterface : function(selected) {
    var board = dojo.byId(this.boardID);
    board.innerHTML = "";
    dojo.require(selected.interface);
    var page = dojo.getObject(selected.interface,true);
    var pageObject = new page();
    board.appendChild(pageObject.domNode);
  },

  launchInterface : function(name)
  {
    var message = dojo.toJson([{ interface_name : name }]);
    ros.callService(this.launchService,message,this.nop);
  },

  nop : function() {},
    
});
