function  batterymeter (){
    this.lock = false
    this.selectPowerMeter=function(capacity)
    {
	var src=null;
	if(capacity >= 88)
	    src='batteryicons/full_battery.png';
	else if(capacity >= 75)
	    src='batteryicons/full_battery_1.png';
	
	else if(capacity >= 63)
	    src='batteryicons/full_battery_2.png';
	
	else if(capacity >= 50)
	    src='batteryicons/full_battery_3.png';
	
	else if(capacity >=38)
	    src='batteryicons/full_battery_4.png';
	else if(capacity >=25)
	    src='batteryicons/full_battery_5.png';
	else if (capacity >=13){
	    src='batteryicons/full_battery_6.png';
	}
	else
	    src='batteryicons/full_battery_red.png';
	return src;
    }
    
    this.handler=function(msg, canvas){
    
	if(this.lock) return;
	
	this.lock=true;
	var img=new Image();	
	img.onload=function(){
	    canvas.clearRect(0,0,173,93);
	    canvas.scale(1,1);
	    canvas.save();
	    canvas.drawImage(img,0,0);
	    canvas.restore();
	}
	var capacity=msg.power_state.relative_capacity;
	img.src=this.selectPowerMeter(capacity);
	this.lock=false;
    }
};