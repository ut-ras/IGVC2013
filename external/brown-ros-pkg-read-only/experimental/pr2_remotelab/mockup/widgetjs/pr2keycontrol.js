function pr2KeyControl (armmessage, armtype, basemessage, basetype, headmessage, headtype){
    this.armmessage=armmessage;
    this.armtype=armtype;
    this.headmessage=headmessage;
    this.headtype=headtype;
    this.basemessage=basemessage;
    this.basetype=basetype;
    this.z= 0;
    this.x = 0;
    this.headX = 0;
    this.headY = 0,
    this.headDeltaX = 0;
    this.headDeltaY = 0;
    this.handX = 0.4;
    this.handDeltaX = 0;
    this.handY = 0.2;
    this.handDeltaY = 0;
    this.handZ = 0.2;
    this.handDeltaZ = 0;
    this.handClosed = false;
    this.handToggle = true;
    this.handXr = .4;
    this.handYr = .2;
    this.handZr = .2;
    this.baseX = 0;
    this.baseZ = 0;
    this.json = function(obj) {return JSON.stringify(obj);},
   

    this.handleKey= function(code, down) {
	switch(code) {
	case 86:
	    //pan left
	    if (down) {
		this.headDeltaX = .01;
	    } else {
		this.headDeltaX = 0;
	    }
	    break;
	case 66:
	    //pan right
	    if (down) {
		this.headDeltaX = -.01;
	    } else {
		this.headDeltaX = 0;
	    }
	break;
	case 78:
	    //tilt up
	    if (down) {
		this.headDeltaY = .01;
	    } else {
		this.headDeltaY = 0;
	    }
	    break;
	case 77:
	    //tilt down
	    if (down) {
		this.headDeltaY = -.01;
	    } else {
		this.headDeltaY = 0;
	    }
	    break;
	case 87:
	    if (down) {
		this.handDeltaX = .1;
	    } else {
		this.handDeltaX = 0;
	    }
	    break;
	case 83:
	    if (down) {
		this.handDeltaX = -.1;
	    } else {
		this.handDeltaX = 0;
	    }
	    break;
	case 68:
	    if (down) {
		this.handDeltaY = -0.1;
	    } else {
		this.handDeltaY = 0;
	    }
	    break;
	case 65:
	    if (down) {
		this.handDeltaY = 0.1;
	    } else {
		this.handDeltaY = 0;
	    }
	    break;
	case 81:
	    if (down) {
		this.handDeltaZ = 0.1;
	    } else {
		this.handDeltaZ = 0;
	    }
	    break;
	case 69:
	    if (down) {
		this.handDeltaZ = -0.1;
	    } else {
		this.handDeltaZ = 0;
	    }
	    break;
	case 37:
	    if (down) {
		this.baseZ = 1;
	    } else {
		this.baseZ = 0;
	    }
	    break;
	case 38:
	    if (down) {
		this.baseX = .5;
	    } else {
		this.baseX = 0;
	    }
	    break;
	case 39:
	    if (down) {
		this.baseZ = -1;
	    } else {
		this.baseZ = 0;
	    }
	    break;
	case 40:
	    if (down) {
		this.baseX = -.5;
	    } else {
		this.baseX = 0;
	    }
	    break;
	case 67:
	    if (down) {
		this.handToggle = true;
		if (this.handClosed) {
		    this.handClosed = false;
		} else {
		    this.handClosed = true;
		}
	    }
	case 81:
	    if (down) {
		this.handX = Math.round(this.handXr*10)/10;
		this.handY = Math.round(this.handYr*10)/10;
		this.handZ = Math.round(this.handZr*10)/10;
	    }
	    break;
	}
    },

    this.look= function(ros,x,y){

	ros.publish(this.headmessage, this.headtype, this.json(
	    {
		'joint_names':["head_pan_joint", "head_tilt_joint"],
		'points':[{
		    'positions':[x,y],
		    'velocities':[0.0, 0.0],
		    'time_from_start':{'nsecs':0,'secs':0},
		}]
	    }
	));
    }
    
   this.hand= function(ros, x,y,z,ox,oy,oz,ow) {
       ros.publish(armmessage,this.armtype,this.json(
	{
	    'header':'{frame_id':'torso_lift_link'}, //may need time
	    'pose':{
		'position':{'x':x,'y':y,'z':z},
		'orientation':{'x':ox,'y':oy,'z':oz,'w':ow},
	    }
	}
	));
    },
    
   
    this.handler= function(ros)
    {
	//console.log('sigh');
	this.headX += this.headDeltaX;
	this.headY += this.headDeltaY;
	if (this.headX < -2.8) {
	    this.headX = -2.8;
	} else {
	    if (this.headX > 2.8) this.headX = 2.8;
	}
	
	if (this.headY < -.24) {
	    this.headY = -.24;
	} else {
	    if (this.headY > 1.16) this.headY = 1.16;
	}
	
	console.log(this.headX);
	this.look(ros, this.headX,this.headY);	


	this.handX += this.handDeltaX;
	this.handY += this.handDeltaY;
	this.handZ += this.handDeltaZ;
	


	this.hand(ros, this.handX, this.handY, this.handZ,0,0,0,1);
	
/*	ros.publish(this.basemessage, this.basetype,this.json(
	    {'linear':	{'x': this.baseX,
			 'y': 0,
			 'z': 0},
	     'angular':	{'x': 0,
			 'y': 0,
			 'z': this.baseZ}}
	));
*/	
	if (this.handToggle) {
	    this.handToggle = false;
	    var position = 0.08;
	    if (this.handClosed) {
		position = -100.00;
		this.handClosed = true;
	    }
	    ros.publish('/l_gripper_controller/command','pr2_controllers_msgs/Pr2GripperCommand', this.json(
		{
		    'position':position,
		    'max_effort':-1.0,
		}
	    ));
	}
	
    }
};

	  
