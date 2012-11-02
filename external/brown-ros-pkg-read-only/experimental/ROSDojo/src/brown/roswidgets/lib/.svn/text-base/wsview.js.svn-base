var WSView = function(dst,w,h) {
    this.dst = dst;
    this.dstCtx = dst.getContext('2d');
    this.dstW = w;
    this.dstH = h;
    this.src = null;
    this.srcCtx = null;
    this.srcData = null;
    this.srcW = 0;
    this.srcH = 0;
    this.srcPixels = null;
}

WSView.prototype.setSize = function(w, h) {
    this.dstW = w;
    this.dstH = h;
}

WSView.prototype.display = function(img) {    
    //get the image data
    var imgW = img.width;
    var imgH = img.height;
    var imgPixels = window.atob(img.data);
    var imgLen = imgPixels.length;
    
    //create new scaling space if needed 
    if (this.src == null  || 
                    this.srcW != imgW || 
                    this.srcH != imgH) {
        var div = document.createElement('div');
        div.innerHTML = '<canvas width="'+imgW+'" height="'+imgH+'"></canvas>';
        this.src = div.firstChild;
        this.srcCtx = this.src.getContext('2d');
        this.srcW = imgW;
        this.srcH = imgH;
        this.srcData = this.srcCtx.getImageData(0,0,this.srcW,this.srcH);
        this.srcPixels = this.srcData.data; 
    }
    
    var i = 0;
    var j = 0;
    while(i < imgLen) {
        for (var k = 0; k < 3; k++) {
            this.srcPixels[j+k] = imgPixels.charCodeAt(i+k);
        }
        if (img.encoding == 'bgr8') {
            var red = this.srcPixels[j];
            this.srcPixels[j] = this.srcPixels[j+2];
            this.srcPixels[j] = red;
        }
        this.srcPixels[j+3] = 255;
        
        if (img.encoding == 'mono8') {
            i += 1;
        } else {
            i += 3;
        }
        j += 4;
    }
    
    this.srcCtx.putImageData(this.srcData,0,0);
    this.dstCtx.save();
    this.dstCtx.scale(this.dstW/this.srcW,this.dstH/this.srcH);
    this.dstCtx.drawImage(this.src,0,0);
    this.dstCtx.restore();

    if (!(img.encoding == 'rgb8' || img.encoding == 'bgr8' || img.encoding=='mono8')) {
        this.setError(img.encoding+" is not a known encoding; the image may be unstable");
    }
    
}

WSView.prototype.setError = function(text) {
    var context = this.dstCtx;
    
    context.fillStyle    = '#000';
    context.font         = '9.5px helvetica';
    context.textBaseline = 'top';
    context.fillText  (text, 10, 10);
}