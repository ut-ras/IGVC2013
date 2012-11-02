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
 
 ros.widgets.ServiceWidget = ros.widgets.Widget.extend({
	 
     init:function(domobj, node){
     // This function gets the document object model object (div element in the html page, i.e. the interface)
     // Puts the text box, drop down lists and other elements on the webpage.
     //this._super(domobj);
     this.serviceList=new Array();
     this.domobj = domobj;
     this.node = node;
    
     // All sensor related services
     this.sensorServices=new Array();
     
     // Service types under sensor services
     // These will be combined into sensorServices array
     this.cameraServices=new Array();
     this.laserServices=new Array();
     this.accimuServices=new Array();
     this.odometryServices=new Array();
     this.touchServices=new Array();

     // All actuator related services
     this.actuatorServices=new Array();

     // Service types under actuator services
     // These will be combined into actuatorServices array
     this.bodyServices=new Array();
     this.headServices=new Array();

     // Array that keeps service types together
     this.serviceTypes=new Array();

     // Coordinate systems related services (tf etc.)
     this.coordinatesysServices=new Array();

     // Network related services
     this.networkServices=new Array();

     // Power and battery related services
     this.powerServices=new Array();

     // Misc. services that don't fit in other groups
     this.otherServices=new Array();

     // Array that keeps all services
     this.allServices=new Array();

     // Array that keeps keywords for grouping the services
     this.kw=new Array();

     // Array that keeps the options of the type select
     this.typeOpts=new Array();

     this.sensorServices.push("\n Sensor related services:");
     this.actuatorServices.push("\n Actuator related services:");
     this.coordinatesysServices.push("\n Coordinate systems related services:");
     this.networkServices.push("\n Network related services:");
     this.powerServices.push("\n Power related services:");
     this.otherServices.push("\n Other services:");

     this.cameraServices.push("\n Services related to cameras:");
     this.laserServices.push("\n Services related to laser range finders and scanners:");
     this.accimuServices.push("\n Services related to accelerometer and IMU:");
     this.odometryServices.push("\n Services related to odometry data:");
     this.touchServices.push("\n Services related to touch sensors:");
     this.bodyServices.push("\n Services related to body of the robot:");
     this.headServices.push("\n Services related to head of the robot:");

     // Keywords for camera related services
     this.kw[0] = /\/wide_stereo/i;
     this.kw[1] = /\/narrow_stereo/i;
     this.kw[2] = /\/r_forearm_cam/i;
     this.kw[3] = /\/l_forearm_cam/i;
     this.kw[4] = /\/camera_synchronizer_node/i;
     this.kw[5]=  /\/prosilica/i;
     this.kw[6] = /\/head_camera/i;
     this.kw[7] = /\/projector_trigger/i;
     this.kw[8] = /\/mjpeg_server/i;

     // Keywords for laser or scanner related services
     this.kw[9] = /\/laser_tilt_controller/i;
     this.kw[10] = /\/tilt_hokuyo_node/i;
     this.kw[11] = /\/base_hokuyo_node/i;

     // Keywords for accelerometer, imu related services
     this.kw[12] = /\/imu_node/i;

     // Keywords for body related services  
     this.kw[13] = /\/torso_controller/i;
     this.kw[14] = /\/l_gripper_controller/i;
     this.kw[15] = /\/r_gripper_controller/i;
     this.kw[16] = /\/pr2_controller_manager/i;

     // Keywords for head related services
     this.kw[17] = /\/head_traj_controller/i;
     this.kw[18] = /\/throttle_head_traj_controller_state/i;

     // Keywords for coordinate system, tf related services
     this.kw[19] = /tf/i;
     this.kw[20] = /\/robot_pose_ekf/i;

     // Keywords for network related services
     this.kw[21] = /broadcaster/i;
     this.kw[22] = /server/i;
     this.kw[23] = /\/network/i;
     this.kw[24] = /\/pr2_ethetCAT/i;
     this.kw[25] = /\/app/i;
  
     // Keywords for power related services
     this.kw[26] = /\/power/i;

     // Keywords for other services
     this.kw[27] = /\/diag_agg/i;
     this.kw[28] = /\/rosout/i;
     this.kw[29] = /\/logger/i;
     this.kw[30] = /\/task/i;
     this.kw[31] = /\/wviz_scene_manager/i;
     this.kw[32] = /\/joy/i;
     this.kw[33] = /\/pr2_run/i;
     this.kw[34] = /\/pr2_mechanism/i;
     this.kw[35] = /\/robot_state/i;
     this.kw[36] = /\/status/i;
     this.kw[37] = /\/realtime/i;


     ////ros_debug(this);
     this.createInterface();
     this.$servicewall= jQuery('#servicewall');

      var that=this;
     this.node.getServices(function(e){that.getServices(e)});

   },
   
   createInterface:function(){
     this.domobj.innerHTML = "<table> <tr> <td valign=\"top\"> <form name=\"filterservices\"> Select a group of services <br> <select id=\"servicegroupmenu\" onchange=\"services_widget.selectGroup(this,1,false)\"> <option value=\"6\">All</option> <option value=\"0\">Sensors</option> <option value=\"1\">Actuators</option> <option value=\"2\">Coordinate Systems</option> <option value=\"3\">Network</option> <option value=\"4\">Power</option> <option value=\"5\">Other</option> </select> <br><br> Select a type of service <br> <select id=\"servicetypemenu\" onchange=\"services_widget.selectType(this,1,false)\"> <option>All</option> </select> <br><br> Search: <input type=\"text\" size=\"10\" id=\"servicesearch\"/><br /> <!--This second input is to resolve the browser quirk that happens with one input field. If the user presses enter, the form is reloaded in case of one input field. To solve it, we have this second invisible input field.--> <input type=\"text\" name=\"resolvequirk\" value=\"Fix browser bug\" style=\"display:none\" /> <button type=\"button\" onclick=\"services_widget.findService()\">Search</button> </form> </td> <td> <form> <textarea id=\"servicewall\" readonly rows=\"20\" cols=\"90\" style=\"resize: none;\"> </textarea> </form> </td> </tr> </table>"
   },
   
   // Gets all published services from the robot
   getServices:function(list){
     ////ros_debug("getServices");
     ////ros_debug(this);
     this.$servicewall= jQuery('#servicewall')
     this.serviceList = list;
     this.$servicewall.attr('value', "\n All Services:\n\n");
     for (var t in this.serviceList){
         ////ros_debug(serviceList[t]);
	 this.$servicewall.attr('value', this.$servicewall.attr('value')+t + ":" + this.serviceList[t] + "\n");
     }
     this.printAllServices();
   },
   
   // This function prints all published services out.
   printAllServices:function(){
     ////ros_debug("printAllServices");
     //this.$servicewall.attr('value', "printAllServices");
     ////ros_debug(this.serviceList);
     for (var t in this.serviceList){
     	     var service = this.serviceList[t];
     	     for(var k in this.kw){
     		 if(8>=k && k>=0){
     		     if(service.match(this.kw[k]) != null){
     			 this.cameraServices.push(service);
     			 this.sensorServices.push(service);
     		     }
     		 }
     		 if(11>=k && k>=9){
     		     if(service.match(this.kw[k]) != null){
     			 this.laserServices.push(service);
     			 this.sensorServices.push(service);
     		     }
     		 }
     		 if(k==12){
     		     if(service.match(this.kw[k]) != null){
     			 this.accimuServices.push(service);
     			 this.sensorServices.push(service);
     		     }
     		 } 
     		 if(16>=k && k>=13){
     		     if(service.match(this.kw[k]) != null){
     			 this.bodyServices.push(service);
     			 this.actuatorServices.push(service);
     		     }
     		 }
     		 if(18>=k && k>=17){
     		     if(service.match(this.kw[k]) != null){
     			 this.headServices.push(service);
     			 this.actuatorServices.push(service);
     		     }
     		 }
     		 if(20>=k && k>=19){
     		     if(service.match(this.kw[k]) != null){
     			 this.coordinatesysServices.push(service);
     		     }
     		 }
     		 if(25>=k && k>=21){
     		     if(service.match(this.kw[k]) != null){
     			 this.networkServices.push(service);
     		     }
     		 }
     		 if(k==26){
     		     if(service.match(this.kw[k]) != null){
     			 this.powerServices.push(service);
     		     }
     		 }
     		 if(37>=k && k>=27){
     		     if(service.match(this.kw[k]) != null){
     			 this.otherServices.push(service);
     		     }
     		 }
     	     }
	 //ros_debug(t + ":" + this.serviceList[t]);      
     }
     
     this.serviceTypes.push(this.cameraServices);
     this.serviceTypes.push(this.laserServices);
     this.serviceTypes.push(this.accimuServices);
     this.serviceTypes.push(this.bodyServices);
     this.serviceTypes.push(this.headServices);

     this.allServices.push(this.sensorServices);
     this.allServices.push(this.actuatorServices);
     this.allServices.push(this.coordinatesysServices);
     this.allServices.push(this.networkServices);
     this.allServices.push(this.powerServices);
     this.allServices.push(this.otherServices);

     for(var g in this.allServices){
     	 for(var t in this.allServices[g]){
     	     if(t != "0"){
     		 this.$servicewall.attr('value', this.$servicewall.attr('value') + t+":"+this.allServices[g][t] + "\n");
     	     }
     	     else{
     		 this.$servicewall.attr('value', this.$servicewall.attr('value') + this.allServices[g][t] + "\n");
     	     }
     	 }
     }

     for(var i=0;i<8;i++){
     	 this.typeOpts.push(document.createElement("option"));
     }
  
     this.typeOpts[0].text="All";
     this.typeOpts[0].value="0"
     this.typeOpts[0].selected=true;
     this.typeOpts[0].defaultSelected=true;

     this.typeOpts[1].text="Cameras";
     this.typeOpts[1].value="1";

     this.typeOpts[2].text="Laser";
     this.typeOpts[2].value="2";

     this.typeOpts[3].text="Acc/IMU";
     this.typeOpts[3].value="3";

     this.typeOpts[4].text="Body";
     this.typeOpts[4].value="4";

     this.typeOpts[5].text="Head";
     this.typeOpts[5].value="5";    
     
   },

   // Filters the published services depending on the selected group by the user (with the first dropdown box)
   selectGroup:function(){
     //ros_debug("Select group");
     /////////////

     this.$servicewall.attr('value', "");
     var choice = jQuery('#servicegroupmenu');
     //var n = choice.attr('options['+choice.attr('selectedIndex')+']','value');
     var n = choice.val();//attr('selectedIndex');
     //ros_debug(n);

     if(n==6){
	 var i=0;
	 var f=5;
     }
     else{
	 var i=n;
	 var f=n;
     }

     for(var g=i;g<=f;g++){
	 for(var t in this.allServices[g]){
	     if(t != 0){
		 this.$servicewall.attr('value', this.$servicewall.attr('value')+ t+": "+this.allServices[g][t] + "\n");
	     }
	     else{
		 this.$servicewall.attr('value', this.$servicewall.attr('value')+ this.allServices[g][t] + "\n" + "\n");
	     }
	 } 
     }

     var tm = jQuery("#servicetypemenu");
     // Clean the type drop down menu
     tm.empty();

     this.typeOpts[0].selected=true;
     this.typeOpts[0].defaultSelected=true;
     this.typeOpts[0].value="0";
     // Add the first option by default.
     tm.append(this.typeOpts[0]);
     
     switch(choice.attr('selectedIndex'))
     {
	     // All types
	 case 0:
	     for( var i=1; i < 6; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;
	     // Sensor types
	 case 1:
	     for( var i=1; i < 4; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;    
	     // Actuator types
	 case 2:
	     for( var i=4; i < 6; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;
     }
     ////////////  
   },
   
   // Filers the published services depending on the selected type, i.e. sub-group. (with the second dropdown box)
   selectType:function(){
	     //ros_debug("Select type");
	     /////////////////////
	     this.$servicewall.attr('value',"");
	     var typeval = jQuery("#servicetypemenu").val();//attr('selectedIndex');
	     //ros_debug(typeval);
	     var groupval = jQuery("#servicegroupmenu").val();//attr('selectedIndex');

	     var filteredResults=new Array();

	     if(typeval==0){
		 switch(Number(groupval))
		     {
		     case 0:
			 filteredResults.push(this.sensorServices);
			 break;
		     case 1:
			 filteredResults.push(this.actuatorServices);
			 break;
		     case 2:
			 filteredResults.push(this.coordinatesysServices);
			 break;
		     case 3:
			 filteredResults.push(this.networkServices);
			 break;
		     case 4:
			 filteredResults.push(this.powerServices);
			 break;
		     case 5:
			 filteredResults.push(this.otherServices);
			 break;
		     }
	     }
	     else{
		 filteredResults.push(this.serviceTypes[typeval-1]);
	     }
  
	     for(t in filteredResults[0]){
		 //ros_debug(filteredResults[0][t]);
		 if( t != 0){
		     this.$servicewall.attr('value', this.$servicewall.attr('value')+ t + ": " + filteredResults[0][t] + "\n");
		 }
		 else{
		     this.$servicewall.attr('value', this.$servicewall.attr('value')+ filteredResults[0][t] + "\n" + "\n");
		 }
	     }
	     //////////

   },
   
   // Filters the services by keywords that the user is searching
   findService:function(){
	     //ros_debug("Search keyword");
	     //////////////////
	     // var key;

	     // if(e.which){

	     // 	 key = (e.keyCode ? e.keyCode : e.which);

	     // 	 //key = e.which ? event.which : window.e.keyCode;
	     // 	 //ros_debug(key);
	     // }
  
	     var searchexp = new RegExp(jQuery('#servicesearch').attr('value'));
	     //ros_debug(jQuery("#servicesearch").attr('value'));
	     var dummyservice;

	     //if(key == 13){
		 this.$servicewall.attr('value',"");
		 for(var g in this.allServices){
		     for(var t in this.allServices[g]){
			 dummyservice = this.allServices[g][t];
			 if(dummyservice.match(searchexp) != null){
			     this.$servicewall.attr('value', this.$servicewall.attr('value')+ dummyservice + "\n");
			 }
		     }
		 }
		 if(this.$servicewall.attr('value')==""){
		     this.$servicewall.attr('value', "No matching service was found.");
		 }
	     //}
	     ///////////

   },   
});
