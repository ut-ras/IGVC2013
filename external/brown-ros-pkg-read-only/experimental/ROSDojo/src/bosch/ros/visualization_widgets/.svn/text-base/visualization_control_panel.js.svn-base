/*******************************************************************************
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2010, Robert Bosch LLC. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. * Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution. * Neither the name of the Robert Bosch nor the names
 * of its contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/


ros.widgets.VisControlPanel = ros.widgets.Widget.extend({
    init: function(domobj) {
	this._super(domobj);
	
	var tool=['<button type="button" , id="vis_panel_open"><center>Visualization Control Panel</center></button>'];
	//    var tool =         ['<ol id="selectable_vis_list">  </ol> <div id="vis_selection_widget" title="Visualization Seletion Widget" class="" objtype=VisSelectionWidget></div> '  ];
	
	
	this.jquery.append(tool.join(''));
	this.node=node;
	this.visualizationwidgetmanager=new ros.remotelabwidgets.VisualizationWidgetManager(node, "vsw_visualizations");
	
	this.displayoptions=[];
	this.displayoptions.push(['Axes', 'AxesNode']);
	this.displayoptions.push(['Camera', "CameraNode"]);
	this.displayoptions.push(['Grid', "GridNode"]);
	this.displayoptions.push(['GridCells', "GridCellsNode"]);
	this.displayoptions.push(['Image', "ImageNode"]);
	this.displayoptions.push(['Interactive Marker', "InteractiveMarkerNode"]);
	this.displayoptions.push(['LaserScan', "LaserScanNode"]);
	this.displayoptions.push(['Map', "MapNode"]);
	this.displayoptions.push(['Marker', "MarkerNode"]);
	this.displayoptions.push(['Path', "PathNode"]);
	this.displayoptions.push(['Pose',"PoseNode"]);
	this.displayoptions.push(['PoseArray', "PoseArrayNode"]);
	this.displayoptions.push(['PointCloud2', "PointCloudNode"]);
	this.displayoptions.push(['Polygon', "PolygonNode"]);
	this.displayoptions.push(['Odometry', "OdometryNode"]);
	this.displayoptions.push(['Range', "RangeNode"]);
	this.displayoptions.push(['RobotModel', "RobotModelNode"]);
	this.displayoptions.push(['TF', "TFNode"]);
	
	this.currentHtml= this.getInitialDisplayHtml();
	//    ros_debug($('#fixedframes_parameter_values'));
	this.globalOptionHtml=this.getInitialOptionHtml();
	ros_debug('ADDING GLOBAL opt html');
	$("#globaloptions_parameters").html(this.globalOptionHtml);
	this.fixedframe='/odom_combined';
	this.targetframe=this.fixedframe;
	
	this.frameSelector=null;
	
	//this.jquery.menu();
	this.setUpCallBacks();
	
    },
    setVM:function(vm){
	this.vm=vm;
    },
    
    setUpCallBacks:function(){
	var that=this;
	
	jQuery('#vis_panel_open').click(function(e){
	    
	    $( "#visualization_dialog" ).dialog({ height: 450,  width: 850 });
	    htmltext=that.currentHtml;//that.getDisplayHtml()
	    $("#visualizations").html(htmltext);

	    that.visualizationwidgetmanager.setupWidgets();

	    //set up the callbacks for the new HTML
	    $('#selectable_vis_list').selectable({
		selected: function(event,ui){
		    name=$(this).find('.ui-selected').attr('id');
		    //text=that.visualizationwidgetmanager.getVisualizationParameterHtml(name);
		    ros_debug(name)
		    //ros_debug(text)
		    //$('#visualization_parameters').html(text);
		    that.selectedname=name;
		}
	    });
	    

	    $('#vis_panel_close').click(function(e){
		// Here we need to check if the user changed the properties of
		// existing visualization nodes. The ones that have been
		// changed should have their redraw attribute set to 1.
		// 
		// If redraw is 1 then we keep the widget (i.e. don't delete the object
		// itself) but trigger the sceneviewer to destroy its mesh and reload
		// the object into the scene.
		
		$('#selectable_vis_list li').each(function(index, element){
		    var name = $(element).attr('id');
		    var sn = that.vm.getSceneNodeByName(name);
		    // The user clicked on Close button of the control panel, let's update our attributes
		    console.log(sn);
		    if( sn.redraw == 1 ){
			
			ros_debug("Redrawing the mesh");

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

			// We drew the visualization node, reset the flag
			sn.redraw = 0;
		    }
		    
		});
		
		$( "#visualization_dialog" ).dialog('close');
	    });    
	    
	    /*opens box to add a new visualization*/
	    jQuery('#vsw_add').click(function(e){
		
		$( "#add_visualization_dialog" ).dialog({ height: 450,  width: 850 });
		htmltext=that.getSelectBoxHtml()
		$("#select_visualization").html(htmltext);
		
		
		//set up the callbacks for the new HTML
		$('#vsw_add_selector').change(function(e){
		    
		    selectedvalue=$("#vsw_add_selector").val();
		    $('#vsw_selected_name').val(selectedvalue+"_"+that.vm.counter);
		});
		
		$('#vsw_submit_type').click(function(e){
		    index=$("#vsw_add_selector").val();
		    for (var i in that.displayoptions){
			if(that.displayoptions[i][0]==index){
			    type=that.displayoptions[i][1];
			    break;
			}
		    }

		    // Get the name of the selected visualization node
		    name=$('#vsw_selected_name').val();
		    
		    // Add the name in the left column of the table. 
		    $('#selectable_vis_list').append('<li class="ui-widget-content" id="' +name+'" >'+ name + "</li>");
		    
		    ros_debug("control panel says: type is " + type);
		    ros_debug("control panel says: name is " + name);
	
		    // This increments the instance counter
		    // pushes the widget type and name in the visualizations array
		    // also keeps the instance of the widget in visualization_hash array
		    //var my_widget = that.visualizationwidgetmanager.add(type, name);
		    var sn = that.vm.addVisualizationNode(type,name);

		    // The node widget is added (e.g. GridWidget) and now
		    // is the time to add a Properties widget that will handle
		    // all the properties of that specific instance of, say, GridWidget.
		    var my_prop_widget = that.visualizationwidgetmanager.add("PropertiesWidget",name+"_properties");

		    // Which visualization widget's property will be set by this property widget?
		    my_prop_widget.sceneNode = sn;
		    var self = this;
		    var f = function(topics){my_prop_widget.availableTopics = topics;};

		    sn.model.node.getTopics(f);
		    
		    // Then let's add a properties button for our visualization widget
		    // In the simplest case this should open a small dialog where we will see the topic textbox of the visualization node
		    my_prop_widget.button = $('<li class="ui-widget-content" id="'+ name + '_properties"><center><button type="button", id="'+name+'_properties_button"><center>'+name+'_properties'+'</center></button></center></li>');

		    // Now let's append the html code for the button in the visualization_parameters div.
		    $('#visualization_parameters').append(my_prop_widget.button);

		    // We also need to set the callbacks for the button we just inserted
		    that.visualizationwidgetmanager.setupWidget(name+"_properties");

		    // This is where we keep the html of the Vis. Control Panel
		    // When click on the button and open it again, it should be
		    // retrieved from this string variable
		    that.currentHtml=$("#visualizations").html();

		    // Finally let's add the object in the scene viewer
		    // vm stands for visualization manager
		    //my_widget.sceneNode = that.vm.addVisualizationNode(my_widget);//.type, my_widget.current_frame, my_widget.size, my_widget.resolution, my_widget.color]);
		    		    
		    // We're done adding, close the dialog.
		    $( "#add_visualization_dialog" ).dialog('close');
		});
		
		$('#vsw_cancel').click(function(e){
		    // Don't add anything, close the dialog
		    $( "#add_visualization_dialog" ).dialog('close');
		    //$("#visualizations").html(htmltext)
		});
		
	    });

	    //Handles a callback to open global options window
	    jQuery('#vsw_globaloptions').click(function(e){
		$( "#globaloptions_dialog" ).dialog({ height: 450,  width: 850 });
		currenthtml=$('#'+that.fixedframeselector).html();
		ros_debug("global options DIALOG");
		ros_debug(currenthtml);
		//$("#globaloptions_parameters").html(that.globalOptionHtml);
	    });
	    
	    //Code to remove a visualization type
	    jQuery('#vsw_remove').click(function(e){
		
		// This is the name of the visualization node we would like to remove
		selectedvalue=$("#vsw_selected_name").val();
		
		//ros_debug(that.selectedname);
		
		// Following will remove the list items from the Vis. Select Window
		// the one in the left column
		$('#' + that.selectedname).remove();
		
		// the one in the right column
		$('#' + that.selectedname + "_properties").remove();

		// From here down we just delete the elements we inserted or added
		// in the callback function of the add button
		
		// For consistency, let's do things in the reverse order we added them

		// First let's remove the button html from the visualization_parameters div.
		// In order to do that, we need to get the handle for that widget
		var sn = that.vm.getSceneNodeByName(that.selectedname);
		var my_prop_widget = that.visualizationwidgetmanager.getVisualizationWidgetByName(that.selectedname+"_properties");
		
		// We kept the html snippet in the attribute of the widget object
		// now we can remove it easily
		my_prop_widget.button.remove();

		// Unsubscribe from the topics if any
		for(var k in sn.keys){
		   if(k == "topic"){
		       //console.log("UNSUBSCRIBING FROM THE TOPIC ON REMOVE");
		       //sn.model.node.unsubscribe(sn.topic, function(msg){ that.updateFromMessage(msg)});
		       sn.unsub();
		   }
		}
		
		// Let's remove the visualization node from the scene viewer
		that.vm.removeVisualizationNode(sn.nodeId);
		
		// Finally let's remove the widgets from the vis. widget manager
		//that.visualizationwidgetmanager.remove(my_widget.type,that.selectedname);
		that.visualizationwidgetmanager.remove(my_prop_widget.type,that.selectedname+"_properties");		

		
		// Lets try deleting the widget's sceneNode and see if we get rid of its callbacks
		delete sn;
		
		// Store the modified html in the variable for when we shall
		// open the Vis. Control Panel		
		that.currentHtml=$("#visualizations").html();
	    });
	    
	    
	});

	
	
    },
    
    
    getInitialOptionHtml:function(){
	var htmlstring='<center><table border="1" width=700> <br> <td> <ol id="parameters"><li>Fixed Frame  <li>Target Frame  </ol></td> <td><div id="fixedframes_parameter_values"></div><br><div id="targetframe_parameter_values"></div> </td></table></center>';
	this.createFixedFrameSelector('fixedframes_parameter_values', 'base_link');
	return htmlstring;
    },
    
    getInitialDisplayHtml:function(){
	
	//	  var htmlstring='';
	
	var htmlstring ='<center><table border="1" width=700> <br> <td> <ol id="selectable_vis_list">  </ol></td> <td><div id="visualization_parameters"></div> </td></table></center>';
	htmlstring=htmlstring+'<div id="vsw_control_button"><center><button type="button", id="vsw_add">  <center>Add</center>  </button><button type="button", id="vsw_remove">  <center>Remove</center>  </button><button type="button", id="vsw_globaloptions">  <center>Global Options</center>  </button></center></div><div id="vsw_visualizations"> </div>';
	//htmlstring=htmlstring+'<br><div id="vis_selection_widget" title="Visualization Seletion Widget" class="" objtype=VisSelectionWidget></div> ';
	htmlstring=htmlstring+'<br><center><button type="button", id="vis_panel_close">  <center>Close</center>  </button>';
	
	//htmlstring=htmlstring+'<form id="vsw_add_selector_name" type="text">Display Name<input type\"text\" id=\"vsw_selected_name\"/> </form>';
	//htmlstring=htmlstring+'<center><button type="button", id="vsw_submit_type">  <center>Add</center>  </button><button type="button", id="vsw_cancel"> <center>Cancel</center></button></center>';
	return htmlstring;
    },
    
    getSelectBoxHtml:function(){
	htmlstring='';
	htmlstring='<select  id="vsw_add_selector" size="'+ this.displayoptions.length + '">';
	for(var i in this.displayoptions) {
	    htmlstring= htmlstring + '<option value = "' + this.displayoptions[i][0] + '"> ' + this.displayoptions[i][0] + ' </option>';
	}
	htmlstring=htmlstring+"</select>";
	
	htmlstring=htmlstring+'<form id="vsw_add_selector_name" type="text">Display Name<input type\"text\" id=\"vsw_selected_name\"/> </form>';
	htmlstring=htmlstring+'<center><button type="button", id="vsw_submit_type">  <center>Add</center>  </button><button type="button", id="vsw_cancel"> <center>Cancel</center></button></center>';
	return htmlstring;
    },

    
    createFixedFrameSelector:function(div_id, default_frame)
    {
	if(this.frameSelector==null){
	    //call constructor
	    
	    this.frameSelector=new ros.remotelabwidgets.Frame_Selector_Widget(this.node);
	    ros_debug('frame selector initialized');
	    //	      ros_debug(this.frameSelector);
	}
	
	var n = arguments.length;
	if(n > 1) {
	    default_frame="fixed_frame";
	}
	
	//get the html list
	this.fixedframeselector=div_id + '_selector';
	that=this;
	this.frameSelector.getBasicSelectorHtml(div_id, default_frame, this.fixedframeselector, function(e){
	    
	    ros_debug("FIXEDFRAMESELECTOR");
	    ros_debug(that.fixedframeselector);
	    ros_debug($('#'+that.fixedframeselector));
	    $('#'+that.fixedframeselector).change(function(e2){
						      
		that.fixedframevalue=$('#'+that.fixedframeselector).val();
		ros_debug('FRAME SELECTOR');
		ros_debug(that.fixedframevalue);
	    });
	});
    },
});
