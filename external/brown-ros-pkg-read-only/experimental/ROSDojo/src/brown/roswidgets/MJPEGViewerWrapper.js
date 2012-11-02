dojo.provide("roswidgets.MJPEGViewerWrapper");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.require("dijit.form.ComboBox");
dojo.require("dijit.form.Button");
dojo.require("dijit.Tooltip");

dojo.require("rosdojo.rosdojo");
dojo.require("roswidgets.common.Utils");

dojo.declare("roswidgets.MJPEGViewerWrapper",[dijit._Widget, dijit._Templated], {
  quality : 100,
  width: 320,
  height: 240,

  templateString : dojo.cache("roswidgets", "templates/MJPEGViewerWrapper.html"),
  defaultUrls: [ "http://localhost:8080/stream?topic=/camera/rgb/image_color?width=320?height=240"],

  postCreate : function() {
    dojo.addClass(this.domNode,"mjpegviewer");
  	//dojo.style(this.domNode, "width", this.width+"px");
   	//dojo.style(this.domNode, "height", this.height+"px");

    // Create the constituent dijit widgets
    var instructions = "Enter the URL of mjpeg server";
        this.urlLabel.title = instructions;
    this.dropdown = new dijit.form.ComboBox({title: instructions},this.dropdownAttach);
    this.startButton = new dijit.form.Button({label:"Start"},this.startAttach);
    this.stopButton = new dijit.form.Button({label:"Stop"}, this.stopAttach);

    // Hide the appropriate parts of the widget
    dojo.style(this.startButton.domNode, "display", "");
    dojo.style(this.stopButton.domNode, "display", "none");

    // Populate the dropdown with previous URL values
    this.setDropdownURLs(this.loadURLs());

    // Connect events
    this.connect(this.dropdown,"onKeyDown","dropdownKeyPressed");
    this.connect(this.startButton,"onClick","startPressed");
    this.connect(this.stopButton, "onClick", "stopPressed");
    
  },

  dropdownKeyPressed : function(e) {
    if(!this.dropdown._opened && e.keyCode == dojo.keys.ENTER) {
      this.dropdown._onBlur();
  //    this.startButton.onClick();
    }
  },

  startPressed : function(event) 
  {
    if(this.dropdown.value) {
      var url = this.dropdown.value;
      this.updateView(url);
      dojo.style(this.startButton.domNode, "display", "none");
      dojo.style(this.stopButton.domNode, "display", "");
    }
  },

  stopPressed : function(event) 
  {
    this.removeImg();
    dojo.style(this.startButton.domNode, "display", "");
    dojo.style(this.stopButton.domNode, "display", "none");
  },
                

  updateView : function(url) 
  {
    console.log("url = " + url);
    if (url) {
      this.createImg(url);
    }
  },

  createImg: function(url) {
      var ctr = document.createElement('center');
      var img = document.createElement('img');
      this.img = img;
      img.src = url;//"http://" + url + ":" + this.port + "/stream?topic=" + this.topic + "?width="+this.width+"?height="+this.height+"?quality="+this.quality;
      img.style.width = this.width+"px";
      img.style.height = this.height+"px";
      ctr.appendChild(img);
      this.domNode.appendChild(ctr);
      this.ctr = ctr;
  },
  
  removeImg: function() {
      this.domNode.removeChild(this.ctr);
  },

  saveURL : function(url) {
      var urls = this.loadURLs();
      
      // First, remove the url from the list if it's already in it
      var i = urls.indexOf(url);
      if (i >= 0) {
          urls.splice(i, 1);
      }
      
      // Then add the url to the start of the list
      urls.splice(0, 0, url);
      
      // Prune the list if it's longer than 5
      if (urls.length > 5) {
          urls.splice(urls.length - 2, 1);
      }
      
      // Finally save the url list as a cookie
      dojo.cookie("mjpegserver_urls", dojo.toJson(urls), {});
  },

  loadURLs : function() {
    var urls = dojo.fromJson(dojo.cookie("mjpegserver_urls"));
    
    if (urls == null) {
      urls = dojo.clone(this.defaultUrls);
    }
    return urls;
  },
    
  setDropdownURLs : function() {
    var urls = this.loadURLs();
    var data = [];
    for ( var i = 0, len = urls.length; i < len; i++) {
        var url = urls[i];
        data.push({
            name : url,
            id : url,
            value : url
        });
    }

    this.dropdown.store.setData(data);
    this.dropdown.setValue(urls[0]);
  },
});
