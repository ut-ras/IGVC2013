dojo.provide("blockbuilder.BlockVisualiser");

dojo.require("blockbuilder.Utils");
dojo.require("blockbuilder.THREEVisualiser");

dojo.declare("blockbuilder.BlockVisualiser", blockbuilder.THREEVisualiser, {
    
    // Canvas variables
    width: 400,
    height: 300,
    
    // Camera variables
    view_angle: 90,
    near: 0.0001,
    far: 4000,
    radius: 1500,
    theta: Math.PI / 4,
    phi: Math.PI / 2,
    
    // Block variables
    numBlocks: 6,
    blockSize: 250,
    
    constructor: function() {
        this.blocks = {};
        this.centerx = -this.blockSize/2;
        this.centerz = -this.blockSize/2;
        this.centery = this.height / 2;
        
        this.inherited(arguments);
    },
    
    postCreate: function() {
        this.inherited(arguments);
        this.drawFloor();
        dojo.addClass(this.domNode, "blockvisualiser");
    },
    
    drawFloor: function() {
        var max = this.numBlocks;
        var start = -(this.numBlocks * this.blockSize) / 2;
        
        for ( var i = 0; i < max; i++) {
            var z = start + i * this.blockSize;
            for ( var j = 0; j < max; j++) {
                var x = start + j * this.blockSize;

                var color = (i+j)%2==0 ? 0xe0e0e0 : 0xb0b0b0;
                
                var plane = new THREE.Mesh(new THREE.PlaneGeometry(this.blockSize, this.blockSize), new THREE.MeshBasicMaterial({ color: color }));
                plane.position.x = x;
                plane.position.z = z;
                plane.position.y = -this.blockSize/2;
                plane.rotation.x = -90 * (Math.PI / 180);
                plane.overdraw = true;
                this.scene.add(plane);                
                
            }
        }
        
    },
    
    addBlock: function(x, y, z) {
        var materials = [];
        for (var i = 0; i < 6; i++) {
            materials.push(new THREE.MeshBasicMaterial({ color: Math.random() * 0xffffff }));
        }
        
        var min = this.numBlocks / 2;
        
        var block = new THREE.Mesh(new THREE.CubeGeometry(this.blockSize, this.blockSize, this.blockSize, 1, 1, 1, materials), new THREE.MeshFaceMaterial());
        block.position.x = (x - min) * this.blockSize;
        block.position.z = (min - y - 1) * this.blockSize;
        block.position.y = z * this.blockSize;
        block.overdraw = true;
        
        this.scene.add(block);
        
        this.blocks[this.blockID(x,y,z)] = block;        
    },
    
    removeBlock: function(x, y, z) {
        var block = this.blocks[this.blockID(x,y,z)];
        if (block) {
            delete this.blocks[this.blockID(x,y,z)];
            this.scene.remove(block);
        }
    },
    
    blockID: function(x, y, z) {
        return x+","+y+","+z;
    }
    
    
});
