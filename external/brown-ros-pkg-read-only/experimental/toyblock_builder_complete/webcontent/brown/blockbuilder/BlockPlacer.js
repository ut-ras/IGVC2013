dojo.provide("blockbuilder.BlockPlacer");

dojo.require("dijit._Widget");

dojo.require("dojox.gfx");

dojo.require("blockbuilder.Utils");

dojo.declare("blockbuilder.BlockPlacer", dijit._Widget, {
    
    numBlocks : 5,
    blockSize : 40,
    border : 10,
    
    gridColour : "black",
    textColour : "black",
    
    colourOrder : [ "#F2F3F4", "#FFBF00", "#FF7E00", "#FF033E", "#9966CC", "#A4C639", "#CD9575" ],
    
    postCreate : function() {
        this.blocks = [];
        
        dojo.addClass(this.domNode, "blockplacer");
        
        var size = this.blockSize * this.numBlocks + 2 * this.border;
        
        this.surface = dojox.gfx.createSurface(this.domNode, size, size);
        this.group = this.surface.createGroup();
        this.drawGrid();
        this.center();
        
        window.ondblclick = function DoubleClick(evt) {
            if (window.getSelection)
                window.getSelection().removeAllRanges();
            else if (document.selection)
                document.selection.empty();
        }

    },
    
    getBlocks: function() {
        var output = [];
        for (var i = 0, len = this.blocks.length; i < len; i++) {
            var block = this.blocks[i];
            for (var j = 0; j < block.clickCount; j++) {
                var position = { x: block.i, y: block.j, z: j};
                var orientation = { x: 0, y: 0.707, z: 0, w: 0.707 };
                output.push({ position: position, orientation: orientation });
            }
        }
        return output;
    },
    
    center : function() {
        var dimensions = this.surface.getDimensions();
        this.group.applyTransform(dojox.gfx.matrix.translate(dimensions.width / 2, dimensions.height / 2));
    },
    
    drawGrid : function() {
        var max = this.numBlocks;
        var start = -(this.numBlocks * this.blockSize) / 2;
        
        for ( var i = 0; i < max; i++) {
            var y = start + i * this.blockSize;
            for ( var j = 0; j < max; j++) {
                var x = start + j * this.blockSize;
                
                var block = this.group.createGroup();
                
                block.i = i;
                block.j = j;
                block.x = x;
                block.y = y;
                block.clickCount = 0;
                block.rectangle = block.createRect({
                    x : x,
                    y : y,
                    width : this.blockSize,
                    height : this.blockSize
                }).setStroke(this.gridColour).setFill(this.colourOrder[0]);
                
                block.rectangle.connect("mouseup", dojo.hitch(this, "blockClicked", block));
                
                this.blocks.push(block);
            }
        }
        
    },
    
    blockClicked : function(block, evt) {
        var x = block.x;
        var y = block.y;
        
        // First, remove the old number on the block
        if (block.numberText) {
            block.numberText.removeShape();
        }
        
        // Add or remove as appropriate
        if (evt.button == 0) {
            this.onBlockAdded(block.i, block.j, block.clickCount);
            block.clickCount++;
        } else if (evt.button == 1 && block.clickCount > 0) {
            block.clickCount--;
            this.onBlockRemoved(block.i, block.j, block.clickCount);
        }
        
        // Add a new number if necessary
        if (block.clickCount > 0) {
            block.numberText = this.group.createText({
                x : x + 10,
                y : y + this.blockSize - 5,
                text : "" + block.clickCount,
                align : "start"
            }).setFont({
                family : "Arial",
                size : "20pt",
                weight : "bold"
            }).setFill(this.textColour);
            block.numberText.connect("mouseup", dojo.hitch(this, "blockClicked", block));
        }
        
        // Set the block colour
        var colour = this.colourOrder[this.colourOrder.length - 1];
        if (block.clickCount < this.colourOrder.length) {
            colour = this.colourOrder[block.clickCount];
        }
        block.rectangle.setFill(colour);
        
    },
    
    // Connect to these events for callbacks
    onBlockAdded: function(x, y, z) {},
    onBlockRemoved: function(x, y, z) {}

});
