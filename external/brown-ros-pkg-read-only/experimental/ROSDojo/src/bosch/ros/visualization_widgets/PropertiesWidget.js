/* Halit Bener Suay - Fixed-Term - Bosch CR-RTC/NA
 * 05/22/2012
*/

/* This widget will receive an existing node's
 * properties and will show a dialog where you can
 * modify them on the fly. Node properties are read
 * from the object itself. Depending on the type of
 * the property, this dialog show one or more of
 * the below:
 * - text input field
 * - drop-down field
 * - color-picker
 * - number scrolling thingy (what is it called again?)
*/

ros.visualization_widgets.PropertiesWidget = Class.extend({
    init: function(mySceneNode) {
	this.type = "PropertiesWidget";
	this.id='';
	this.dialog_html='';
	this.button='';
	this.sceneNode=mySceneNode;
	this.h_space=0;
	this.v_space=0;
	this.availableTopics=[];
	this.name='';
    },

    padding: function(str) {
	var pad = "";
	if(str.length < 6){
	    for(var i=str.length; i < 6; i=i+1){
		pad = pad + "0";
	    }
	}
	var padded = pad + str;
	return padded;
    },

    getHtml: function(name){

	this.name = name;
  this.saveID = name + '_saveButton';
  this.cancelID = name + '_cancelButton';
  console.log('here');
	
	// Just in case redraw was set to 1 previously reset it.
	this.sceneNode.redraw=0;

	// This is where we generate the html code
	// for the properties dialog
	if(this.dialog_html == ''){

	    // Here iterate through the properties of the visualization widget and 
	    // add appropriate form elements in the dialog's html
	    // Let's begin from generating the title
	    //this.dialog_html= '<div id="'+name+'_dialog" title="'+name+'">';

      var dialog = document.createElement('div');
      dialog.setAttribute('id',name+'_dialog');
      dialog.setAttribute('title',name);
      var css = document.createElement('style');
      css.type = 'text/css';
      css.innerHTML = '#leftpane {float: left; width: 330px; height: 200px;}#rightpane {float: left; width: 50px; height: 200px;}#form { width: 500px; height: 200px;}#label { float: left; width: 100px; }#textbox { float: left; width: 200px; }#picker { width: 0px; height: 0px; }#choosetopic {float: left; width: 30px; }#slide { width: 30px; height: 100px; }#palette { float: left; height: 20px; width: 20px; }';
      dialog.appendChild(css);

	    // Now let's go through the keys of whatever widget we are trying to show properties
	    // and put a text box for those keys
	    // console.debug(this.sceneNode.keys);

	    // Start generating a form
	    //this.dialog_html=this.dialog_html+'<div id="form"> <form>';
      var formdiv = document.createElement('div');
      var formform = document.createElement('form');
      formdiv.setAttribute('id','form');
      formdiv.appendChild(formform);
      dialog.appendChild(formdiv);
	    
	    // Put the left pane here
	    //this.dialog_html=this.dialog_html+'<div id="leftpane">';
      var leftpanediv = document.createElement('div');
      leftpanediv.setAttribute('id','leftpane');
      formform.appendChild(leftpanediv);
	    
	    for(var k in this.sceneNode.keys){
		// k is the index string; the name of the property.
		// this.sceneNode.keys[k] returns the value of the property.
		if( k == "color"){

		    // Convert object's color into hex
		    var rgbInHex = this.sceneNode.color[2]*255 | ( this.sceneNode.color[1]*255<< 8) | (this.sceneNode.color[0]*255 << 16);

		    // Set palette color
		    
		    // This input box is special, in that, it's not clickable (readonly)
		    // and that its background color depends on the color of the vis. wdgt.
        var label = document.createElement('div');
        label.setAttribute('id','label');
        label.innerHTML = k;
        
        var textboxdiv = document.createElement('div');
        textboxdiv.setAttribute('id','textbox');

        var input = document.createElement('input');
        input.setAttribute('type','text');
        input.setAttribute('name',name + '_'+k+'_input');
        input.setAttribute('readonly','readonly');
        input.setAttribute('style',"background-color:#'+rgbInHex.toString(16)+'; color:#'+rgbInHex.toString(16)+';");
        input.setAttribute('value',this.sceneNode.keys[k]);
        input.style.backgroundColor = '#'+rgbInHex.toString(16);
        input.style.color = '#'+rgbInHex.toString(16);
        this.colorinput = input;
        console.log(input);
        textboxdiv.appendChild(input);

        leftpanediv.appendChild(label);
        leftpanediv.appendChild(textboxdiv);
		}
		else if( k == "topic"){
		    // If the property of the widget is topic then put a small button next to it
		    // When the user clicks on the button it will pop the list of available topics
        var label = document.createElement('div');
        label.setAttribute('id','label');
        label.innerHTML = k;

        var textboxdiv = document.createElement('div');
        textboxdiv.setAttribute('id','textbox');

        var input = document.createElement('input');
        input.setAttribute('type','text');
        input.setAttribute('name',name + '_'+k+'_input');
        input.setAttribute('value',this.sceneNode.keys[k]);
        textboxdiv.appendChild(input);

        leftpanediv.appendChild(label);
        leftpanediv.appendChild(textboxdiv);
		}
		else{
		    // If the property of the widget is not "color"
		    // then just add its label and a textbox
		    //this.dialog_html=this.dialog_html +'<div id="label">'+ k + '</div> <div id="textbox"> <input type="text" name="'+name+'_'+k+'_input" 
        //  value="'+this.sceneNode.keys[k]+'"> </div>';

        var label = document.createElement('div');
        label.setAttribute('id','label');
        label.innerHTML = k;

        var textboxdiv = document.createElement('div');
        textboxdiv.setAttribute('id','textbox');

        var input = document.createElement('input');
        input.setAttribute('type','text');
        input.setAttribute('name',name + '_'+k+'_input');
        input.setAttribute('value',this.sceneNode.keys[k]);
        textboxdiv.appendChild(input);

        leftpanediv.appendChild(label);
        leftpanediv.appendChild(textboxdiv);
		}
	    }

	    // Put a rightpane here
	    //this.dialog_html=this.dialog_html+'<div id="rightpane">';
      var rightpane = document.createElement('div');
      rightpane.setAttribute('id','rightpane');
      formform.appendChild(rightpane);

	    if("color" in this.sceneNode.keys){
		// Rainbow slide yay!
      this.picker = document.createElement('div');
      rightpane.setAttribute('id','picker');
      rightpane.appendChild(this.picker);

      this.slide = document.createElement('div');
      rightpane.setAttribute('id','slide');
      rightpane.appendChild(this.slide);
        
	    }
	    
  	    // Add a Save and a Cancel button and finish generating the div 
	    // first </div> is for div class="form", id: name_properties_form
	    // second </div> is for div id: name_properties_dialog
      var saveButton = document.createElement('button');
      saveButton.setAttribute('type','button');
      saveButton.setAttribute('id',this.saveID);
      saveButton.innerHTML = '<center>Save</center>';
      dialog.appendChild(saveButton);

      var cancelButton = document.createElement('button');
      cancelButton.setAttribute('type','button');
      cancelButton.setAttribute('id',this.cancelID);
      cancelButton.innerHTML = '<center>Cancel</center>';
      dialog.appendChild(cancelButton);

      this.dialog_html = dialog;
      this.dialog = dialog;
	}	
  console.log(this.colorinput);
	// If the scene node has a color property exposed,
	// Open the color picker's rainbow slide here
  var that = this;
	if("color" in this.sceneNode.keys){
	    ColorPicker(this.slide,
		
		// old
		//document.getElementById('palette');
		
		// new
		// For the color textbox we don't have an id, so we get it by its name
		this.colorinput,
		
		function(hex, hsv, rgb) {
		    
		    console.log("COLOR PICKER SAYS hex is:");
		    console.log(hex);
        console.log(this.colorinput);

		    
		    //old
		    //document.getElementById('palette').style.backgroundColor=hex;
		    that.colorinput.style.backgroundColor=hex;
		    
		    that.colorinput.style.color=hex;
		    that.colorinput.value=[Number(rgb.r/255), Number(rgb.g/255), Number(rgb.b/255)];
		    
		    //console.log(rgb);
		    //ros_debug(Number(rgb.r/255));
		    //ros_debug(Number(rgb.g/255));
		    //ros_debug(Number(rgb.b/255));
		    
		    that.sceneNode.color = [Number(rgb.r/255), Number(rgb.g/255), Number(rgb.b/255)];
		    that.sceneNode.redraw=1;
		}
	    );
	}	
  console.log('done');
	return this.dialog_html;
    },

    onOpen: function(){
	
  console.log('yoyo');
	// Just in case redraw was set to 1 previously reset it.
	this.sceneNode.redraw=0;

	// Refresh the input text fields
	for(var k in this.sceneNode.keys){
	    if( k == "color"){

		// Convert object's color into hex
		var rgbInHex = this.sceneNode.keys[k][2]*255 | (this.sceneNode.keys[k][1]*255 << 8) | (this.sceneNode.keys[k][0]*255 << 16);

		
		// Convert the rgb value in hex and pad the string with zeros
		// if it's shorter than 6 letters.
		rgbInHex = this.padding(rgbInHex.toString(16));
		
		console.log("When opening the dialog RGB Color value converted in Hex");
		console.log(rgbInHex)
      this.colorinput.style.backgroundColor = "#"+rgbInHex;
		  this.colorinput.style.color = "#"+rgbInHex;
		  this.colorinput.value = this.sceneNode.keys[k];
	    }
	    else{
		document.getElementsByName(this.name+'_'+k+'_input')[0].value = this.sceneNode.keys[k];
	    }
	    //console.log(document.getElementsByName(name+'_'+k+'_input')[0]);
	}
  console.log('yoyoyo');
    },

    onClose: function(){
	// What to do?
    },

    onSave: function(){
	    console.log("asdfasdfasdfasdf");
	
	this.sceneNode.redraw=1;
	
	//this.sceneNode.resolution = Number($('#'+name+'_'+k+'properties_dialog input').val());
	//console.log(Number($('#'+name+'_properties_dialog input').val()));

	// Let's iterate through our textboxes and save their values into our variables
	for(var k in this.sceneNode.keys){


	    // Get the widget's attribute's type
	    var attrType = Object.prototype.toString.call(this.sceneNode.keys[k]).match(/^\[object (.*)\]$/)[1];
	    
	    
	    //eval('this.sceneNode.'+this.sceneNode.keys[k]+'='+document.getElementsByName(name+'_'+k+'_input')[0].value);		    
	    var my_eval_string = "this.sceneNode."+k+"=";
	    
      console.log(this.dialog_html);
	    console.log("Spitting the current value of: "+this.name+"_"+k+"_input");
	    console.log(document.getElementsByName(this.name+'_'+k+'_input'));
	    console.log("ATTR NAME:");
	    console.log(k);

	    if( attrType == "Array" ){

		console.log("ATTR TYPE is Array");	

		my_eval_string = my_eval_string + "[" + document.getElementsByName(this.name+"_"+k+"_input")[0].value.toString() + "];";
		var my_array = document.getElementsByName(this.name+"_"+k+"_input")[0].value.split(",");
		// TODO We should check the array length in a smart way
		var key_value = [Number(my_array[0]),Number(my_array[1]),Number(my_array[2])];
		
	    }
	    else if(attrType == "String" ){
		console.log("ATTR TYPE is String");
		
		my_eval_string = my_eval_string +"\"" +document.getElementsByName(this.name+"_"+k+"_input")[0].value.toString() +"\";";
		var key_value = document.getElementsByName(this.name+"_"+k+"_input")[0].value.toString();
	    }
	    else if(attrType == "Number") {
		console.log("ATTR TYPE is Number");
		my_eval_string = my_eval_string + "Number("+document.getElementsByName(this.name+"_"+k+"_input")[0].value.toString() +");";
		var key_value = Number(document.getElementsByName(this.name+"_"+k+"_input")[0].value);
	    }
	    
	    //console.log("Trying to evaluate the following:");
	    //console.log(my_eval_string);

	    // This will update the attributes of our visualization widget object
	    eval(my_eval_string);

	    // We should also update the keys of our visualization widget object
	    // Becasue this is where we read the key values from when we open
	    // the dialog
	    this.sceneNode.keys[k] = key_value;
	    
	}
    },
    
    onCancel: function(){
	// What to do?
    },
    
});
