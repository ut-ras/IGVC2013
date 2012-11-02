dojo.global.THREE.SphereMountedCamera = function(fov, aspect, near, far, radius, theta, phi, centerx, centery, centerz) {
    
    THREE.PerspectiveCamera.call( this, fov, aspect, near, far );
    
    this.theta = theta !== undefined ? theta : Math.PI / 2.0;
    this.phi = phi !== undefined ? phi : 0.0;
    this.radius = radius !== undefined ? radius : 1000;
    this.centerx = centerx !== undefined ? centerx : 0;
    this.centery = centery !== undefined ? centery : 0;
    this.centerz = centerz !== undefined ? centerz : 0;
    
    this.origin = new THREE.Vector3(centerx, centery, centerz);
    
    this.recalculateLocation();
    
};

dojo.global.THREE.SphereMountedCamera.prototype = new THREE.PerspectiveCamera();
dojo.global.THREE.SphereMountedCamera.prototype.constructor = THREE.SphereMountedCamera;

dojo.global.THREE.SphereMountedCamera.prototype.recalculateLocation = function() {
    if (this.theta > (Math.PI / 2.0)) {
        this.theta = (Math.PI / 2.0);
    }
    if (this.theta < 0) {
        this.theta = 0;
    }
    
    this.position.x = this.radius * Math.sin(this.phi) * Math.sin(this.theta) + this.centerx;
    this.position.y = this.radius * Math.cos(this.theta) + this.centery;
    this.position.z = this.radius * Math.cos(this.phi) * Math.sin(this.theta) + this.centerz;
    
    this.lookAt(this.origin);
};