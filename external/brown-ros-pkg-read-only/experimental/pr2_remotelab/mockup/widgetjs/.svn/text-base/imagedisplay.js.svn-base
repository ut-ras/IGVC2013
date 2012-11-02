function imagedisplay (){
    this.lock=false;

    this.handler=function(msg, video_status, canvas){
	if (this.lock) return;
	
	if(!video_status) return;
	
	this.lock = true;
	var img = new Image();
	img.onload = function() {
	    canvas.clearRect(0,0,640,480);
	    canvas.save();
	    canvas.scale(5,5);
	    canvas.drawImage(img,0,0);
	    canvas.restore();
	}

	img.src = msg.uri;
	this.lock=false;
    }

    
};

