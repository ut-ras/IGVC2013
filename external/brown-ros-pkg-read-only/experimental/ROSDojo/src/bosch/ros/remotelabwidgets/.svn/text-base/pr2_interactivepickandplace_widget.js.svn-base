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

ros.widgets.pr2_interactivepickandplace_widget=Class.extend({
    init:function(node, divID, vm){
	this.node=node;
	this.divID=divID;
	this.divIDAdvancedOpts="advancedOpts";
	this.vm=vm;
	//	this.pickandplace_manager= new ros.pickandplace.PickAndPlaceManager(node,vm);
//	this.create_initial_divHTML:function();
	
	this.divbuttonID='buttondiv';	
	this.advancedoptions=true;
	this.currarm='l';  //start with left
	this.arm_motion_arm='r';
	this.arm_motion_direction='side';
	this.arm_motion_method='planner';
	this.sensing_type='collision_object';
	
	this.grasp_planning_service_name="/grasp_planning_gripper_click";
	
	this.pickup_action_name="/object_manipulator/object_manipulator_pickup";
	pickup_client_spec=new ros.actionlib.ActionSpec('/object_manipulation_msgs/PickupAction');
	this.pickup_client = new ros.actionlib.SimpleActionClient(node, this.pickup_action_name, pickup_client_spec);

	this.place_action_name="/object_manipulator/object_manipulator_place";
	place_client_spec=new ros.actionlib.ActionSpec('/object_manipulation_msgs/PlaceAction');

	this.place_client = new ros.actionlib.SimpleActionClient(node, this.place_action_name, place_client_spec);
	

	console.log(ros);
	this.mech_interface =new ros.pr2interaction.pr2_mechanism_interface(node);
	this.statusLabel=" ";
	this.lock=false;  //lock will be used to disallow functionality when 

	this.setButtonText();			
	
	this.create_divHTML();
    },



    setButtonText:function(){
	this.GoButton1ID="go1_click";
	this.GoButton1_on_text=" <p><button type=\"button\", id=\"go1_click\" style=\"width: 200px;\"> GO</button></p>";
	this.GoButton1_disabled_text=" <p><button type=\"button\", id=\"go1_click\" style=\"width: 200px;\" disabled> GO</button></p>";

	this.headButtonID="head_click";
	this.headButton_on_text=" <p><button type=\"button\", id=\"head_click\" style=\"width: 200px;\"> Look at table</button></p>";
	this.headButton_disabled_text=" <p><button type=\"button\", id=\"head_click\" style=\"width: 200px;\" disabled> Look at table</button></p>";

	this.placeID="place_click";
	this.place_on_text=" <p><button type=\"button\", id=\"place_click\" style=\"width: 200px;\"> Place Object</button></p>";
	this.place_disabled_text=" <p><button type=\"button\", id=\"place_click\" style=\"width: 200px;\" disabled> Place Object</button></p>";

	this.graspID="grasp_click";
	this.grasp_on_text=" <p><button type=\"button\", id=\"grasp_click\" style=\"width: 200px;\"> Grasp Object</button></p>";
	this.grasp_disabled_text=" <p><button type=\"button\", id=\"grasp_click\" style=\"width: 200px;\" disabled> Grasp Object </button></p>";
	
	this.staticmapID="staticmap_click";
	this.staticmap_status=true;
	this.staticmap_on_text=" <p><button type=\"button\", id=\"staticmap_click\" style=\"width: 200px;\"> Take new static map</button></p>";
	this.staticmap_disabled_text=" <p><button type=\"button\", id=\"staticmap_click\" style=\"width: 200px;\" disabled> Take new static map</button></p>";

	this.resetmapID="resetmap_click";
	this.resetmap_status=true;
	this.resetmap_on_text=" <p><button type=\"button\", id=\"resetmap_click\" style=\"width: 200px;\"> Reset map</button></p>";
	this.resetmap_disabled_text=" <p><button type=\"button\", id=\"resetmap_click\" style=\"width: 200px;\" disabled> Reset map</button></p>";

     },
    
    create_divHTML:function(){	

	if(this.currarm=='l')
	    div_text="<input type=\"radio\" name=\"selected_arm\" value=\"l\" checked/> Left Arm </label>   <input type=\"radio\" name=\"selected_arm\" value=\"r\"/> Right Arm </label> ";
	else
	    div_text="<input type=\"radio\" name=\"selected_arm\" value=\"l\" /> Left Arm </label>   <input type=\"radio\" name=\"selected_arm\" value=\"r\" checked/> Right Arm </label> ";
	    
	
	    div_text=div_text+this.create_buttonHTML();
	
	
  
	//Set up advanced option text here.
	
	div_text=div_text+"<h3>Advanced Options</h3><input type=\"checkbox\" name=\"advanced_options\" value=\"default\" checked/>Use defaults <br><div id="+this.divIDAdvancedOpts+">";

	
        console.log(div_text);
	$('#'+this.divID).html(div_text);
	
	this.advancedOptsHtml();

	this.setUpCallBacks();
	
    },


    disableButtons:function(){

	    jQuery('#go1_click').attr('disabled', true);
	    jQuery('#head_click').attr('disabled', true);
	    jQuery('#place_click').attr('disabled', true);
	    jQuery('#grasp_click').attr('disabled', true);
	    jQuery('#staticmap_click').attr('disabled', true);
	    jQuery('#resetmap_click').attr('disabled', true);
			
	},
    
    enableButtons:function(){
	    jQuery('#go1_click').attr('disabled', false);
	    jQuery('#head_click').attr('disabled', false);
	    jQuery('#place_click').attr('disabled', false);
	    jQuery('#grasp_click').attr('disabled', false);
	    jQuery('#staticmap_click').attr('disabled', false);
	    jQuery('#resetmap_click').attr('disabled', false);
			
	
	},

    setUpCallBacks:function(){
	var that=this;

	jQuery("input[name='selected_arm']").change(function(e){
	    console.log('radio button change');
	    arm=jQuery("input[name='selected_arm']:checked").val();
	    
	    that.currarm=arm;
      });

	jQuery("input[name='advanced_options']").change(function(e){
	    console.log('advanced options change');
	    on=jQuery("input[name='advanced_options']").is(':checked');
	    that.advancedoptions=on;
	    that.advancedOptsHtml()
      });
	jQuery('#arm_motion').change(function(e){
		console.log('arm_motion changed');
		this.arm_motion_arm=jQuery('#arm_motion option:selected').val();
	});

	jQuery('#arm_direction_motion').change(function(e){
		console.log('arm_direction_motion changed ');
		this.arm_motion_direction=jQuery('#arm_diection_motion option:selected').val();
	    });
	jQuery('#arm_method_motion').change(function(e){
		console.log('arm motion method changed');
		this.arm_motion_method=jQuery('#arm_method_motion option:selected').val();
	    });
	jQuery('#sensing').change(function(e){
		console.log('collision change');
		this.sensing_type=jQuery('#sensing option:selected').val();
	    });
	
	jQuery('#go1_click').click(function(e){
	    console.log('Moving Arm');
	    that.armMotion();
	});
	
	jQuery('#head_click').click(function(e){
	    console.log('Moving Head');
	    that.moveHead();
	});
	
	jQuery('#place_click').click(function(e){
	    console.log('Placing object using gripper click.');
	    that.placeObject();    
	});

	jQuery('#grasp_click').click(function(e){
	    console.log('Grasping object using gripper click.');
	    that.graspObject()
	});

	jQuery('#staticmap_click').click(function(e){
	    console.log('Acquiring the new static map');
	    that.newStaticMap();
	});

	jQuery('#resetmap_click').click(function(e){
	    console.log('Resetting the static map');
	    that.resetStaticMap();
	});
	
    },


 create_buttonHTML:function(){
	div_text="<div id=\""+this.divbuttonID + "\">";
	    
	div_text=div_text+this.grasp_on_text;
	
	div_text=div_text+this.place_on_text;
	

	div_text=div_text+"<br> <h3> Sensing </h3> <select id=\"sensing\"><option id=\"collision_object\">collision objects</option><option id=\"attatched_objects\">attached objects</option> <option id=\"collision_map\">collision map</option></select>";

	div_text=div_text+this.staticmap_on_text+this.resetmap_on_text ;

	div_text=div_text+"<h3> Helper Functions</h3> Arm motion: <br><select id=\"arm_motion\"><option value=\"r\">right</option><option value=\"l\">left</option></select><select id=\"arm_direction_motion\"><option value=\"side\">to side </option> <option value=\"front\"> to front</option><option value=\"open\">open gripper</option><option value=\"close\">close gripper</option></select><select id=\"arm_method_motion\"><option value=\"planner\">with planner</option><option value=\"openloop\">open loop</option></select>";

	div_text=div_text+this.GoButton1_on_text;
	div_text=div_text+"Head Motion:";
	div_text=div_text+this.headButton_on_text;
	return div_text;
//	$('#'+this.divID).html(div_text);

    },
    
    advancedOptsHtml:function(){
	    console.log(this.advancedoptions)
	    if(this.advancedoptions){
		$('#'+this.divIDAdvancedOpts).html(" ");
	    }
	    else{
		string= "<input type=\"checkbox\" name=\"reactive_grasping\" value=\"reactive_value\"/>reactive grasping <br>After grasp try to lift <select id=\"reactive_value\"><option value=\"10\">10<option value=\"9\">9</option> <option value=\"8\">8</option> <option value=\"7\">7</option><option value=\"6\">6</option><option value=\"5\">5</option><option value=\"4\">4</option><option value=\"3\">3</option><option value=\"2\">2</option><option value=\"1\">1</option></select> cm <br> <input type=\"checkbox\" name=\"reactiveforce\" value=\"reactiveforce_value\"/>reactive gripper force <br>After place try to retreat  <select id=\"force_value\"><option value=\"10\">10<option value=\"9\">9</option> <option value=\"8\">8</option> <option value=\"7\">7</option><option value=\"6\">6</option><option value=\"5\">5</option><option value=\"4\">4</option><option value=\"3\">3</option><option value=\"2\">2</option><option value=\"1\">1</option></select> cm <br> <input type=\"checkbox\" name=\"reactive_place\" value=\"place_value\"/>reactive place <br> Lift/Place:<select id=\"direction\"><option value=\"vertical\">vertically</option><option value=\"approach\">along approach direction</option></select>";
		$('#'+this.divIDAdvancedOpts).html(string);
	    }
	    
    },
    

    armMotion:function(){
	var that=this;
	console.log("this functionality is being  implemented");

	if(this.arm_motion_direction=='side'){
	    if(this.arm_motion_method=='planner'){
		this.statusLabel="moving arm to side using motion planner";
		
		
	    }
	}
	
	
    },

    moveHead:function(){
	var that=this;
	console.log("this functionality is not currently implemented");
	
    },

    placeObject:function(){
	    var that=this;
	    console.log("this functionality is not currently implemented");

    },

    graspObject:function(){
	var that=this;
	console.log("this functionality is not currently implemented");

    },

    newStaticMap:function(){
	var that=this;
	console.log("this functionality is not currently implemented");

    },

    resetStaticMap:function(){
	var that=this;
	console.log("this functionality is not currently implemented");
	
    },

});
