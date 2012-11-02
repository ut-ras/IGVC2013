/*******************************************************************************
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2011, Robert Bosch LLC. All rights reserved.
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

//\Author Ralf Kempf, Robert Bosch LLC
 
 ros.widgets.LoggerWidget = ros.widgets.Widget.extend({
	 
     init:function(domobj, node){
     // This function gets the document object model object (div element in the html page, i.e. the interface)
     // Puts the text box, drop down lists and other elements on the webpage.
     //this._super(domobj);
     this.topicList=new Array();
     this.domobj = domobj;
     this.node = node;
    
     // All sensor related topics
     this.sensorTopics=new Array();
     
     // Topic types under sensor topics
     // These will be combined into sensorTopics array
     this.cameraTopics=new Array();
     this.laserTopics=new Array();
     this.accimuTopics=new Array();
     this.odometryTopics=new Array();
     this.touchTopics=new Array();

     // All actuator related topics
     this.actuatorTopics=new Array();

     // Topic types under actuator topics
     // These will be combined into actuatorTopics array
     this.bodyTopics=new Array();
     this.headTopics=new Array();

     // Array that keeps topic types together
     this.topicTypes=new Array();

     // Coordinate systems related topics (tf etc.)
     this.coordinatesysTopics=new Array();

     // Network related topics
     this.networkTopics=new Array();

     // Power and battery related topics
     this.powerTopics=new Array();

     // Misc. topics that don't fit in other groups
     this.otherTopics=new Array();

     // Array that keeps all topics
     this.allTopics=new Array();

     // Array that keeps keywords for grouping the topics
     this.kw=new Array();

     // Array that keeps the options of the type select
     this.typeOpts=new Array();
	
	 //Array for filtered and selected Topics
	 this.filteredResults=new Array();
	 this.dummytopic;
	 this.selectedTopics=new Array();

     this.sensorTopics.push("\n Sensor related topics:");
     this.actuatorTopics.push("\n Actuator related topics:");
     this.coordinatesysTopics.push("\n Coordinate systems related topics:");
     this.networkTopics.push("\n Network related topics:");
     this.powerTopics.push("\n Power related topics:");
     this.otherTopics.push("\n Other topics:");

     this.cameraTopics.push("\n Topics related to cameras:");
     this.laserTopics.push("\n Topics related to laser range finders and scanners:");
     this.accimuTopics.push("\n Topics related to accelerometer and IMU:");
     this.odometryTopics.push("\n Topics related to odometry data:");
     this.touchTopics.push("\n Topics related to touch sensors:");
     this.bodyTopics.push("\n Topics related to body of the robot:");
     this.headTopics.push("\n Topics related to head of the robot:");

     // Keywords for camera related topics
     this.kw[0] = /\/wide_stereo/i;
     this.kw[1] = /\/narrow_stereo/i;
     this.kw[2] = /\/r_forearm_cam/i;
     this.kw[3] = /\/l_forearm_cam/i;
     this.kw[4] = /\/camera_synchronizer_node/i;
     this.kw[5]=  /\/prosilica/i;
     this.kw[6] = /\/head_camera/i;
     this.kw[7] = /\/projector_trigger/i;

     // Keywords for laser or scanner related topics
     this.kw[8] = /\/laser_tilt_controller/i;
     this.kw[9] = /\/base_scan/i;
     this.kw[10] = /\/tilt_hokuyo_node/i;
     this.kw[11] = /\/base_hokuyo_node/i;

     // Keywords for accelerometer, imu related topics
     this.kw[12] = /\/accelerometer/i;
     this.kw[13] = /\/torso_lift_imu/i;

     // Keywords for odometry topics
     this.kw[14] = /\/base_odometry/i;

     // Keywords for touch sensor topics
     this.kw[15] = /\/pressure/i;

     // Keywords for body related topics  
     this.kw[16] = /\/motor_trace/i;
     this.kw[17] = /\/l_arm_controller/i;
     this.kw[18] = /\/r_arm_controller/i;
     this.kw[19] = /\/torso_controller/i;
     this.kw[20] = /\/l_gripper_controller/i;
     this.kw[21] = /\/r_gripper_controller/i;
     this.kw[22] = /\/base_controller/i;
     this.kw[23] = /\/mechanism_statistics/i;
     this.kw[24] = /\/joint_states/i;

     // Keywords for head related topics
     this.kw[25] = /\/head_traj_controller/i;

     // Keywords for coordinate system, tf related topics
     this.kw[26] = /\/tf/i;
     this.kw[27] = /\/robot_pose_ekf/i;

     // Keywords for network related topics
     this.kw[28] = /\/ping/i;
     this.kw[29] = /\/users_online/i;
     this.kw[30] = /\/network/i;
     this.kw[31] = /\/pr2_ethetCAT/i;
     this.kw[32] = /\/app/i;
     this.kw[33] = /\/diagnostics/i;
  
     // Keywords for power related topics
     this.kw[34] = /\/power/i;
     this.kw[35] = /\/battery/i;

     // Keywords for other topics
     this.kw[36] = /\/rosout/i;
     this.kw[37] = /\/ddwrt/i;
     this.kw[38] = /\/calibration/i;
     this.kw[39] = /\/scene/i;
     this.kw[40] = /\/joy/i;
     this.kw[41] = /\/calibrated/i;
     this.kw[42] = /\/dashboard/i;

	 //create the dialog box for selecting the topics
	 this.$dialog = $('<div></div>')
				.html ("<table> <tr> <td valign=\"top\"> <form name=\"filtertopics\"> Select a group of topics <br> <select id=\"topicgroupmenu\" onchange=\"logger_widget.selectGroup(this,1,false)\"> <option value=\"6\">All</option> <option value=\"0\">Sensors</option> <option value=\"1\">Actuators</option> <option value=\"2\">Coordinate Systems</option> <option value=\"3\">Network</option> <option value=\"4\">Power</option> <option value=\"5\">Other</option> </select> <br><br> Select a type of topic <br> <select id=\"topictypemenu\" onchange=\"logger_widget.selectType(this,1,false)\"> <option>All</option> </select> <br><br> Search: <input type=\"text\" size=\"10\" id=\"topicsearch\"/><br /> <!--This second input is to resolve the browser quirk that happens with one input field. If the user presses enter, the form is reloaded in case of one input field. To solve it, we have this second invisible input field.--> <input type=\"text\" name=\"resolvequirk\" value=\"Fix browser bug\" style=\"display:none\" /> <button type=\"button\" onclick=\"logger_widget.findTopic()\">Search</button> <br><br> Selected Topics: <br> <div align=\"center\" > <form> <textarea id=\"selectedTwo\" readonly rows=\"10\" cols=\"23\" style=\"resize: none;\"> </textarea> </form> <br> <br> <br> <button type=\"button\" onclick=\"logger_widget.closeDialog()\" style=\"width:80px;height:32px;\">Ok</button> </div> </form> </td> <td> <form> <div id=\"topicwall\"> </div> </form> </td> </tr> </table>")
				.dialog({
						modal: true,
						autoOpen: false,
						title: 'Select the topics you want to record.',
						height: 700,
						width: 900
				});

     //create some html stuff
     this.createInterface();
     this.$topicwall= jQuery('#topicwall');
	 this.$status = document.getElementById('status');

	 recording = false;
	
	 //Create SimpleActionClient
	 this.action_spec = new ros.actionlib.ActionSpec('topic_logger/TopicLoggerAction');
	 this.client = new ros.actionlib.SimpleActionClient(this.node,'/topicLogger',this.action_spec);
	 //Wait for server startup
	 this.client.wait_for_server(10, function(e){
		if(!e) {
			log("Couldn't find action server for Topic Logging.");
			return;
		}
		log("Connected to TopicLoggerActionServer.");
	  });

	 //get the published topics
     var that=this;
     this.node.getTopics(function(e){that.getTopics(e)});

   },
   
   //creates the Interface on the webpage (Select Topics button, shows selected Topics, Start and Stop buttons, status and the downloadlink
   createInterface:function(){
     this.domobj.innerHTML = "<br> <button type=\"button\" onclick=\"logger_widget.selectTopics()\" style=\"width:200px;height:23px;\">Select Topics</button> <br> <br> Selected Topics: <br> <form> <textarea id=\"selected\" readonly rows=\"10\" cols=\"25\" style=\"resize: none;\"> </textarea> </form> <br> <br> <button type=\"button\" onclick=\"logger_widget.startLogging()\" style=\"width:200px;height:23px;\"> Start Logging </button>  <br> <br> <button type=\"button\" onclick=\"logger_widget.stopLogging()\" style=\"width:200px;height:23px;\"> Stop Logging </button> <br><br>Status:<br> <div id=\"status\"> </div> <br> <br> <div id=\"downloadlink\"> </div>"
   },
   
   // Gets all published topics from the robot
   getTopics:function(list){
     this.$topicwall= document.getElementById('topicwall')
     this.topicList = list;
	 this.topicList.sort();
     this.printAllTopics();
   },

   //function to show and select published Topics in a jQuery dialog
   selectTopics:function(){
		this.showTest2();
		this.$dialog.dialog('open');
   },

   //Close the topic select dialog
   closeDialog:function(){
		this.$dialog.dialog('close');
   },

   //function is called when pressing "Start Logging" button
   startLogging:function(){
		var client = this.client; //is needed for some reason to call get_result()
		if (this.selectedTopics.length<1){
			alert("Please select at least\n1 topic to start recording");
			return
		}
		//Create a goal to send to the action server
		var goal = new Object();	
		goal.command = "start";
		goal.ID = "topicLog";
		goal.selectedTopics = this.selectedTopics;

		//Send goal to action server
		log("Sending start goal to action server.");
		this.$status.innerHTML = "Recording ... <br>";
		//sends the goal and registers the callback for published feedbacks
		client.send_goal(goal, function(e,f){}, function(){}, function(feedback){
			//Show status of Logging
			var size = feedback.filesize / 1048576;
			this.$status = document.getElementById('status');
			this.$status.innerHTML = "Current filesize:" + size.toFixed(3) + " MB <br>";
		});
		recording = true;
	},

   //function is called when pressing "Stop Logging" button
   stopLogging:function(){
		if(!recording){
			alert("You first have to start\nlogging before stoping it");
			return
		}
		//Create a goal to send to the action server
		var client = this.client; //is needed for some reason to call get_result()
		var goal = new Object();
		goal.command = "stop";
		goal.ID = "topicLog";
		goal.selectedTopics = ""

		//Send goal to action server
		log("Sending stop goal to action server.");
		client.send_goal(goal);

		//Show Link to .bag file
		log("Waiting for results from ActionServer.");
		client.wait_for_result(100,function(e) {
			if(!e) {
				log("Didn't receive results of executing action server.");
				return;
			}
			var result = client.get_result(); //get target_filename and link of the bag file
			this.$status = document.getElementById('status');
			this.$status.innerHTML = "Stopped Logging.";
			//log("Saved to file: " + result.target_filename);
			this.$downloadlink = document.getElementById('downloadlink')
			this.$downloadlink.innerHTML = this.$downloadlink.innerHTML + "<br> <a href=\"" + result.downloadURL + "\">" + result.target_filename + "</a>"
		});
		recording = false;
	},
   
   // This function prints all published topics out.
   printAllTopics:function(){
     for (var t in this.topicList){
     	     var topic = this.topicList[t];
     	     for(var k in this.kw){
     		 if(7>=k && k>=0){
     		     if(topic.match(this.kw[k]) != null){
     			 this.cameraTopics.push(topic);
     			 this.sensorTopics.push(topic);
     		     }
     		 }
     		 if(11>=k && k>=8){
     		     if(topic.match(this.kw[k]) != null){
     			 this.laserTopics.push(topic);
     			 this.sensorTopics.push(topic);
     		     }
     		 }
     		 if(13>=k && k>=12){
     		     if(topic.match(this.kw[k]) != null){
     			 this.accimuTopics.push(topic);
     			 this.sensorTopics.push(topic);
     		     }
     		 }
     		 if(14==k){
     		     if(topic.match(this.kw[k]) != null){
     			 this.odometryTopics.push(topic);
     			 this.sensorTopics.push(topic);
     		     }
     		 } 
     		 if(15==k){
     		     if(topic.match(this.kw[k]) != null){
     			 this.touchTopics.push(topic);
     			 this.sensorTopics.push(topic);
     		     }
     		 }
     		 if(24>=k && k>=16){
     		     if(topic.match(this.kw[k]) != null){
     			 this.bodyTopics.push(topic);
     			 this.actuatorTopics.push(topic);
     		     }
     		 }
     		 if(25==k){
     		     if(topic.match(this.kw[k]) != null){
     			 this.headTopics.push(topic);
     			 this.actuatorTopics.push(topic);
     		     }
     		 }
     		 if(27>=k && k>=26){
     		     if(topic.match(this.kw[k]) != null){
     			 this.coordinatesysTopics.push(topic);
     		     }
     		 }
     		 if(28>=k && k>=33){
     		     if(topic.match(this.kw[k]) != null){
     			 this.networkTopics.push(topic);
     		     }
     		 }
     		 if(35>=k && k>=34){
     		     if(topic.match(this.kw[k]) != null){
     			 this.powerTopics.push(topic);
     		     }
     		 }
     		 if(42>=k && k>=36){
     		     if(topic.match(this.kw[k]) != null){
     			 this.otherTopics.push(topic);
     		     }
     		 }
     	     }    
     }
     
     this.topicTypes.push(this.cameraTopics);
     this.topicTypes.push(this.laserTopics);
     this.topicTypes.push(this.accimuTopics);
     this.topicTypes.push(this.odometryTopics);
     this.topicTypes.push(this.touchTopics);
     this.topicTypes.push(this.bodyTopics);
     this.topicTypes.push(this.headTopics);

     this.allTopics.push(this.sensorTopics);
     this.allTopics.push(this.actuatorTopics);
     this.allTopics.push(this.coordinatesysTopics);
     this.allTopics.push(this.networkTopics);
     this.allTopics.push(this.powerTopics);
     this.allTopics.push(this.otherTopics);

	 //create list with checkboxes
	 var l = 0;
	 var html = "";
     for(var g in this.allTopics){
     	 for(var t in this.allTopics[g]){
     	     if(t != "0"){
			 var tmp = this.allTopics[g][t];
			 html = html + "<input type=\"checkbox\" name=\"topic\" value=\"" + this.allTopics[g][t] + "\" onclick=\"logger_widget.showTest("+l+","+g+","+t+")\"> " + this.allTopics[g][t] + "<br>";
     	     }
     	     else{
			 html = html + this.allTopics[g][t] + "<br>";
     	     }
     	 }
     }
	 this.$topicwall.innerHTML = "";
	 this.$topicwall.innerHTML = html;

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

     this.typeOpts[4].text="Odometry";
     this.typeOpts[4].value="4";

     this.typeOpts[5].text="Touch";
     this.typeOpts[5].value="5";

     this.typeOpts[6].text="Body";
     this.typeOpts[6].value="6";

     this.typeOpts[7].text="Head";
     this.typeOpts[7].value="7";    
     
   },

	showTest2:function(){
	  this.$selected = jQuery('#selected');
	  this.$selectedTwo = jQuery('#selectedTwo');
	  this.$selected.attr('value', "");
	  this.$selectedTwo.attr('value', "");
	  for (var i in this.selectedTopics){
	  this.$selected.attr('value', this.$selected.attr('value') + i + ":" + this.selectedTopics[i] + "\n");
	  this.$selectedTwo.attr('value', this.$selectedTwo.attr('value') + i + ":" + this.selectedTopics[i] + "\n");
	  }
	},
	
	//shows the selected topics and adds/deletes topics from list selectedTopics
	showTest:function(l,g,t){
	  var topic="";
	  var exists=false
      if(l==0){
	  	topic = this.allTopics[g][t];
		for (var k in this.selectedTopics){
			if(topic==this.selectedTopics[k]){
				exists=true;
				this.selectedTopics.splice(k,1);
			}
		}
		if(exists!=true){
	  		this.selectedTopics.push(topic);
		}
	  }
      else{
	  	topic = this.filteredResults[0][l];
		for (var j in this.selectedTopics){
			if(topic==this.selectedTopics[j]){
				exists=true;
				this.selectedTopics.splice(j,1);
			}
		}
		if(exists!=true){
	  		this.selectedTopics.push(topic);
		}
	  }

	  this.$selected = jQuery('#selected');
	  this.$selectedTwo = jQuery('#selectedTwo');
	  this.$selected.attr('value', "");
	  this.$selectedTwo.attr('value', "");
	  for (var i in this.selectedTopics){
	  this.$selected.attr('value', this.$selected.attr('value') + i + ":" + this.selectedTopics[i] + "\n");
	  this.$selectedTwo.attr('value', this.$selectedTwo.attr('value') + i + ":" + this.selectedTopics[i] + "\n");
	  }
	},

   // Filters the published topics depending on the selected group by the user (with the first dropdown box)
   selectGroup:function(){
	 this.$topicwall.innerHTML = "";
     var choice = jQuery('#topicgroupmenu');
     var n = choice.val();

	 var l = 0;
     if(n==6){
	 var i=0;
	 var f=5;
     }
     else{
	 var i=n;
	 var f=n;
     }

	 //create the list with the topics and ckeckboxes and check which topic is already selected
	 var checked0=false;
	 var html = "";
     for(var g=i;g<=f;g++){
	 for(var t in this.allTopics[g]){
		 checked0=false;
	     if(t != 0){
			for (var m in this.selectedTopics){		 	
				if (this.allTopics[g][t]==this.selectedTopics[m]){
					html = html + "<input type=\"checkbox\" name=\"topic\" value=\"" + this.allTopics[g][t] + "\" checked=\"ckecked\" onclick=\"logger_widget.showTest("+l+","+g+","+t+")\"> " + this.allTopics[g][t] + "<br>";
					checked0=true;
				}
			}
			if(checked0==false){
					html = html + "<input type=\"checkbox\" name=\"topic\" value=\"" + this.allTopics[g][t] + "\" onclick=\"logger_widget.showTest("+l+","+g+","+t+")\"> " + this.allTopics[g][t] + "<br>";
			}
	     }
	     else{
		 html = html + this.allTopics[g][t] + "<br>";
	     }
	 } 
     }

	 this.$topicwall.innerHTML = "";
	 this.$topicwall.innerHTML = html;

     var tm = jQuery("#topictypemenu");
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
	     for( var i=1; i < 8; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;
	     // Sensor types
	 case 1:
	     for( var i=1; i < 6; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;    
	     // Actuator types
	 case 2:
	     for( var i=6; i < 8; i++){
		 this.typeOpts[i].value = i;
		 tm.append(this.typeOpts[i]);
	     }
	     break;
     }
   },
   
   // Filers the published topics depending on the selected type, i.e. sub-group. (with the second dropdown box)
   selectType:function(){
		 this.$topicwall.innerHTML = "";
	     var typeval = jQuery("#topictypemenu").val();
	     var groupval = jQuery("#topicgroupmenu").val();

		 this.filteredResults=new Array();

	     if(typeval==0){
		 switch(Number(groupval))
		     {
		     case 0:
			 this.filteredResults.push(this.sensorTopics);
			 break;
		     case 1:
			 this.filteredResults.push(this.actuatorTopics);
			 break;
		     case 2:
			 this.filteredResults.push(this.coordinatesysTopics);
			 break;
		     case 3:
			 this.filteredResults.push(this.networkTopics);
			 break;
		     case 4:
			 this.filteredResults.push(this.powerTopics);
			 break;
		     case 5:
			 this.filteredResults.push(this.otherTopics);
			 break;
		     }
	     }
	     else{
		 this.filteredResults.push(this.topicTypes[typeval-1]);
	     }

		 //create list of topics with checkboxes and check which topics are already selected
  		 var checked1=false;
		 var l = 1;
		 var html = "";
	     for(t in this.filteredResults[0])
		 {
			 checked1=false;
			 if( t != 0)
			 {
				for (var n in this.selectedTopics)
				{
					if (this.filteredResults[0][t]==this.selectedTopics[n])
					{
						 html = html + "<input type=\"checkbox\" name=\"topic\" value=\"" + this.filteredResults[0][t] + "\" checked=\"checked\" onclick=\"logger_widget.showTest("+t+")\"> " + this.filteredResults[0][t] + "<br>";
						 checked1=true;
					}
				}
				if(checked1==false)
				{
						html = html + "<input type=\"checkbox\" name=\"topic\" value=\"" + this.filteredResults[0][t] + "\" onclick=\"logger_widget.showTest("+t+")\"> " + this.filteredResults[0][t] + "<br>";
			 	}
			 }
			 else
			 {
				 html = html + this.filteredResults[0][t] + "<br>";
			 }
	     }
		 this.$topicwall.innerHTML = html;

   },
   
   // Filters the topics by keywords that the user is searching
   findTopic:function(){
	     var searchexp = new RegExp(jQuery('#topicsearch').attr('value'));
	     this.dummytopic="";
		 
		 //create a list of topics with checkboxes and check which topics are already selected
		 var html = "";
		 var checked2=false;
		 var l = 0;
		 this.$topicwall.innerHTML = "";
		 for(var g in this.allTopics){
		     for(var t in this.allTopics[g]){
			 checked2=false;
			 this.dummytopic = this.allTopics[g][t];
			 if(this.dummytopic.match(searchexp) != null){
				 for (var h in this.selectedTopics){
					if (this.dummytopic==this.selectedTopics[h]){
						 html = html + "<input type=\"checkbox\" name=\"topic\" onclick=\"logger_widget.showTest("+l+","+g+","+t+")\" checked=\"checked\" value=\"" + this.dummytopic + "\"> " + this.dummytopic + "<br>";
						 checked2=true;
					}
				 }
				 if(checked2==false){
						html = html + "<input type=\"checkbox\" name=\"topic\" onclick=\"logger_widget.showTest("+l+","+g+","+t+")\" value=\"" + this.dummytopic + "\"> " + this.dummytopic + "<br>";
				 }
			 }
		     }
		 }
		 this.$topicwall.innerHTML = html;
		 if(this.$topicwall.innerHTML==""){
		     this.$topicwall.innerHTML = "No matching topic was found.";
		 }
   },  

});
