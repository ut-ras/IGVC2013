dojo.provide("blockbuilder.THREEVisualiser");

dojo.require("blockbuilder.lib.Three", true);
dojo.require("blockbuilder.lib.SphereMountedCamera", true);

dojo.require("dijit._Widget");

dojo.declare("blockbuilder.THREEVisualiser", dijit._Widget, {
    
    clicking: false,
    lastX: 0,
    lastY: 0,
    
    constructor: function() {
        // Just create the scene
        this.createScene();
    },
    
    postCreate: function() {
        // Attach the scene
        this.attachScene();

        // Connect mouse event callbacks for camera
        dojo.connect(this.renderer.domElement, "mouseout", this, "mouseOut");
        dojo.connect(this.renderer.domElement, "mousedown", this, "mouseDown");
        dojo.connect(this.renderer.domElement, "mouseup", this, "mouseUp");
        dojo.connect(this.renderer.domElement, "mousemove", this, "mouseMove");
    },
    
    startup: function() {
        this._animate = dojo.hitch(this, "animate");
        this.animate();
    },
    
    createScene: function() {
        this.scene = new THREE.Scene();
        
        // Add a sphere mounted camera, passing any values that may have been set by clients
        this.camera = new THREE.SphereMountedCamera(this.view_angle, this.width / this.height, this.near, this.far, this.radius, this.theta, this.phi, this.centerx, this.centery, this.centerz);
        this.scene.add(this.camera);
        
        // Create the renderer
        this.renderer = new THREE.CanvasRenderer();
        this.renderer.setSize(this.width, this.height);
                
    },
    
    attachScene: function() {        
        // Attach the renderer
        this.domNode.appendChild(this.renderer.domElement);
    },
    
    mouseDown: function(evt) {
        this.clicking = true;
        this.lastX = evt.clientX;
        this.lastY = evt.clientY;
    },
    
    mouseMove: function(evt) {
        if (this.clicking) {
            var deltaX = evt.clientX - this.lastX;
            var deltaY = evt.clientY - this.lastY;
            this.camera.theta -= deltaY / 100.0;
            this.camera.phi -= deltaX / 100.0;
            this.lastX = evt.clientX;
            this.lastY = evt.clientY;
        }
    },
    
    mouseUp: function(evt) {
        this.clicking = false;
    },
    
    mouseOut: function(evt) {
        this.clicking = false;
    },
    
    _animate: null,    
    animate: function() {
        requestAnimationFrame(this._animate);
        this.render();
    },
    
    render: function() {
        this.camera.recalculateLocation();
        this.renderer.render(this.scene, this.camera);
    },
    
    displayGLError: function() {
        // If browser doesn't support web GL, show the appropriate error
        this.domNode.innerHTML = "";
        
        var div = document.createElement('div');
        div.className = "webglerror border";
        
        var text = document.createTextNode("Cannot initialise WebGL");
        div.appendChild(text);
        
        this.domNode.appendChild(div);       
        
        dojo.style(this.domNode, "width", this.width+"px");
        dojo.style(this.domNode, "height", this.height+"px");

        alert("WebGL is not available in your browser.");
    },
    
});
