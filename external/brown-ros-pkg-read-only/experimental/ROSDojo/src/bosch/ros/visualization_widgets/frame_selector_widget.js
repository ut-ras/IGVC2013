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

/**
 * Code that creates a drop down menu of available tf frames
 * 
 * @class
 * @augments Class
 */

ros.remotelabwidgets.Frame_Selector_Widget=Class.extend({
	init:function(node){
	    ros_debug('initializing frame selector widget');
	    this.node=node;
	    this.tfListerService=this.node.serviceClient("/tf_lister/request_list");
	     this.getListFromServer();
	    this.tfList=null;
	    
	    ros_debug(this.tfList)
	},
   
	/**
	 *Return selector with only the 
	 *
	 */
	getBasicSelectorHtml:function(div_id, default_frame, frameselector, callback){

	    selectorid=frameselector;

	    if(this.tfList==null)
		{
		    var that=this;
		    ros_debug('getting list from server');
		    this.tfListerService.call(ros.json([]), function(e){
			    ros_debug('callback');
			    that.tfList=e.tf_list;
			    ros_debug(e);
			    html='<select id="' + selectorid + '\">' ;
			    
			    for(index=0; index<that.tfList.length; index++)
				{
				    currentframe=that.tfList[index];
				    html=html+'<option value="'+currentframe+'">'+currentframe+'</option>';
				}
			    $("#"+div_id).html(html);
			    //that.setUpCallbacks(div_id);
			    ros_debug($('#'+div_id));
			    ros_debug(callback);
			    callback(e);
			});

		    
		}
	    else{
	 
		html='<select id="' + selectorid + '\">' ;
	    
		for(index=0; index<this.tfList.length; index++)
		    {
			currentframe=this.tfList[index];
			html=html+'<option value="'+currentframe+'">'+currentframe+'</option>';
		    }
		
		$("#"+div_id).html(html);
		//	this.setUpCallbacks(div_id);
	    }

	},


	/**
	 *  Return selector with fixed frame as the first option
	 *
	 */
	getAugmentedSelectorHtml:function(div_id,  default_frame, frameselector, callback){
	    selectorid=frameselector;
	    if(this.tfList==null)
                {
		    var that=this;
		    

		    this.tfListerService.call(ros.json([]), function(e){
			    ros_debug('callback');
			    that.tfList=e.tf_list;
			    ros_debug(e);
			    ros_debug('#'+div_id);
			    ros_debug($('#'+div_id));
			    ros_debug('getting list from server');
			    ros_debug(that.tfList);
			    htmlstring='<select id="' + selectorid + '\">' ;
			    htmlstring=htmlstring+'<option value="fixedframe"> Fixed Frame</option>';
			    for(index=0; index<that.tfList.length; index++)
				{
				    currentframe=that.tfList[index];
				    htmlstring=htmlstring+'<option value="'+currentframe+'">'+currentframe+'</option>';
				}
			    htmlstring=htmlstring+'</optgroup></select>';
			    ros_debug(htmlstring);
			    $('#'+div_id).html(htmlstring);
			    ros_debug($('#'+div_id));
			    //			    that.setUpCallbacks(div_id);
			    callback(e);
			});

		}
	    else{
		htmlstring='<select id="' + selectorid + '\">' ;
		htmlstring=htmlstring+'<option value="fixedframe"> Fixed Frame</option>';
		for(index=0; index<this.tfList.length; index++)
		    {
			currentframe=this.tfList[index];
			htmlstring=htmlstring+'<option value="'+currentframe+'">'+currentframe+'</option>';
		    }
		//		ros_debug(html);
	       		$("#"+div_id).html(html);
		//		this.setUpCallbacks(div_id);

            }
	    return selectorid;
	},

	setUpCallbacks:function(div_id){
	    ros_debug("in callback setup");
	},

	getListFromServer:function(){
	    var that=this;
	    ros_debug('getting list from server');
	    this.tfListerService.call(ros.json([]), function(e){
		    ros_debug('callback');
		    that.tfList=e;
		    ros_debug(e);
		});
	},

	/**	wait:function(){	

	    if(this.tfList==null){

		setTimeout(this.wait(), 10000);
		return;
	    }
	    },**/
});
