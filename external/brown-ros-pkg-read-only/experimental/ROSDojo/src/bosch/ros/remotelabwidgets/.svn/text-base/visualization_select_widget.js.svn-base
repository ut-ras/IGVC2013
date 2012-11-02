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

//Simple run stop widget to wirelessly disable and enable the robot

VisSelectionWidget = ros.widgets.Widget.extend({
  init: function(domobj) {
    this._super(domobj);
  
    var tool = 
          ['<div id="vsw_buttons"><center><button type="button", id="vsw_add">  <center>Add</center>  </button><button type="button", id="vsw_remove">  <center>Remove</center>  </button></center></div>',
           ];
    	
    this.jquery.append(tool.join(''));
    this.haltMotorsService = node.serviceClient("/pr2_etherCAT/halt_motors");
    this.resetMotorsService = node.serviceClient("/pr2_etherCAT/reset_motors");
    this.visualizatonwidgetmanager=new ros.remotelabwidgets.VisualizationWidgetManager(node);
    
    this.displayoptions=[];
    this.displayoptions.push(['Axes', 'AxesWidget']);
    this.displayoptions.push(['Camera', "CameraWidget"]);
    this.displayoptions.push(['Grid', "GridWidget"]);
    this.displayoptions.push(['Grid Cells', "GridCellsWidget"]);
    this.displayoptions.push(['Image', "ImageWidget"]);
    this.displayoptions.push(['Interactive Marker', "InteractiveMarkerWidget"]);
    this.displayoptions.push(['Laser Scan', "LaserScanWidget"]);
    this.displayoptions.push(['Map', "MapWidget"]);
    this.displayoptions.push(['Marker', "MarkerWidget"]);
    this.displayoptions.push(['Path', "PathWidget"]);
    this.displayoptions.push(['Pose',"PoseWidget"]);
    this.displayoptions.push(['Pose Array', "PoseArrayWidget"]);
    this.displayoptions.push(['Point Cloud(2)', "PointCloudWidget"]);
    this.displayoptions.push(['Polygon', "PolygonWidget"]);
    this.displayoptions.push(['Odometry', "OdometryWidget"]);
    this.displayoptions.push(['Range', "RangeWidget"]);
    this.displayoptions.push(['Robot Model', "RobotModelWidget"]);
    this.displayoptions.push(['TF', "TFWidget"]);
   
    //this.jquery.menu();
    this.setUpCallBacks();
    
  },

  setUpCallBacks:function(){
		var that=this;
		
		jQuery('#vsw_add').click(function(e){
			
			 $( "#add_visualization_dialog" ).dialog({ height: 450,  width: 850 });
			 htmltext=that.getDisplayHtml()
			 $("#select_visualization").html(htmltext);
			 
			 
			 //set up the callbacks for the new HTML
			 $('#vsw_add_selector').change(function(e){
					
				 	selectedvalue=$("#vsw_add_selector").val();
				    $('#vsw_selected_name').val(selectedvalue)
			    });
			 
			 $('#vsw_submit_type').click(function(e){
				 index=$("#vsw_add_selector").val();
				 for (var i in this.displayoptions){
					 if(this.displayoptions[i][0]==index){
						 type=this.displayoptions[i][1];
						 break;
					 }
				 }
				 name=$('#vsw_selected_name').val();
				 
				 
				 //that.visualizatonwidgetmanager.add(type, name);
				 $( "#add_visualization_dialog" ).dialog('close');
			 });

			 $('#vsw_cancel').click(function(e){
                  $( "#add_visualization_dialog" ).dialog('close');
			 });
			 
		});

	
		
		jQuery('#vsw_remove').click(function(e){
			
		
		});
		
		
		
  },
  
  getDisplayHtml:function(){
	  
	  htmlstring='';
	  htmlstring='<select  id="vsw_add_selector" size="'+ this.displayoptions.length + '">'
	  for(var i in this.displayoptions) {
	  	htmlstring= htmlstring + '<option value = "' + this.displayoptions[i][0] + '"> ' + this.displayoptions[i][0] + ' </option>';
	  }
	  htmlstring=htmlstring+"</select>"
	  
	  htmlstring=htmlstring+'<form id="vsw_add_selector_name" type="text">Display Name<input type\"text\" id=\"vsw_selected_name\"/> </form>';
	  htmlstring=htmlstring+'<center><button type="button", id="vsw_submit_type">  <center>Add</center>  </button><button type="button", id="vsw_cancel"> <center>Cancel</center></button></center>';
	  return htmlstring;
  },
 
});
