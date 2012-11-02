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

 ros.widgets.SoundWidget = ros.widgets.Widget.extend({

	 init: function(domobj, widgetID){

	     // Create the widget interface (buttons)
	     this.domobj = domobj;
	     this.widgetID= widgetID;
	     this.createInterface();
	     //this.soundOn=false;

	     // Check file compability
	     this.myAudio = document.createElement('audio'); 
	     if (this.myAudio.canPlayType) {
      
		 // Currently canPlayType(type) returns: "", "maybe" or "probably" 
		 this.canPlayMp3 = !!this.myAudio.canPlayType && "" != this.myAudio.canPlayType('audio/mpeg');
		 this.canPlayOgg = !!this.myAudio.canPlayType && "" != this.myAudio.canPlayType('audio/ogg; codecs="vorbis"');
		 this.canPlayWav = !!this.myAudio.canPlayTupe && "" != this.myAudio.canPlayTupe('audio/x-wav');
	     }

	     // Willow Garage has wav, ogg and aiff of all sounds in their library, but not MP3 (at least for now) that's why we're using only the following two.
	     if(this.canPlayWav){
		 this.format = "wav";
	     }
	     else if(this.canPlayOgg){
		 this.format = "ogg";
	     }

	 },

	 createInterface: function(){     

	     this.domobj.innerHTML = "<input id=\"soundWidgetOnButton\" type=\"radio\" name=\"sound\" value=\"on\" onchange=\""+this.widgetID+".soundOn()\"/> Sound On <input type=\"radio\" name=\"sound\" value=\"off\" checked onchange=\""+this.widgetID+".soundOff()\"/> Sound Off <br> <button type=\"button\", id=\"soundTestButton\" style=\"width: 200px;\" disabled=true onclick=\""+this.widgetID+".play('test')\"> Test</button>"
	 },

	 soundOn: function(){
	     ros_debug("sound on");
	     //this.soundOn=true;
	     jQuery('#soundTestButton').attr('disabled',false);		 
	 },

	 soundOff: function(){
	     ros_debug("sound off");
	     //this.soundOn=false;
	     jQuery('#soundTestButton').attr('disabled',true);
	 },

	 // Other widgets will pass different events to this function and the event -> sound mapping is done in switch case.
	 play: function(event){
	     if(jQuery('#soundWidgetOnButton').attr('checked')){
	     
		 switch(event)
		     {
	     
		     case "test":
		     this.fn = "C/CMrnful3";
		     break;

		     case "objectsDetected-AllKnown":
		     this.fn = "C/CKaching3";
		     break;

		     case "objectsDetected-SomeUnknown":
		     this.fn = "C/CMrnful";
		     break;

		     case "objectsDetected-AllUnknown":
		     this.fn = "C/CBmmmm";
		     break;

		     case "oneMomentPlease":
		     this.fn = "B/BOneMomentPlease";
		     break;

		     case "tableDetected":
		     this.fn = "G/G11";
		     break;
		     
		     default:
		     this.fn = "C/C3BeepMed";
		     }
		 this.fn = "http://hri.willowgarage.com/sounds/"+this.fn+"."+this.format;
		 this.snd = new Audio(this.fn);
		 this.snd.play();   
	     }
	 },
 });