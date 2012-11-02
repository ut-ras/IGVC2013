dojo.provide("misc.App");

dojo.require("dijit._Widget");
dojo.require("dijit._Templated");

dojo.declare("misc.App", [dijit._Widget, dijit._Templated ], {
    
    templateString : dojo.cache("misc", "templates/App.html"),
    
    // Required parameters
    title: "",
    link : "",

    // Optional parameters
    img : "",
    wiki : "",
    page : "",
    
    postCreate : function() {
        // Image, Link and Title are required, and handled by the template
        
        // Add the page link, if there is one
        if (this.page) {
            this.addSubTitle("Project Page", this.page);
        }
        
        if (this.wiki) {
            this.addSubTitle("Wiki", this.wiki);
        }
    },
    
    addSubTitle: function(label, url) {
        var div = document.createElement('div');
        dojo.addClass(div, "demosubtitle");
        
        var a = document.createElement('a');
        a.href = url;
        div.appendChild(a);
        
        a.appendChild(document.createTextNode(label));

        this.subtitlesAttach.appendChild(div);
    }
});
