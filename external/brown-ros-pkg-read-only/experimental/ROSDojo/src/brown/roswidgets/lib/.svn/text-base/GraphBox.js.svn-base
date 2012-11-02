"use strict";

var GraphBox = (function() {
  function elem(tagName, parentNode, className) {
    var elem = document.createElement(tagName);
    if (parentNode) parentNode.appendChild(elem);
    if (className) elem.className = className;
    return elem;
  }

  function text(text, parentNode) {
    var elem = document.createTextNode(text);
    if (parentNode) parentNode.appendChild(elem);
    return elem;
  }

  function on(element, name, callback) {
    element.addEventListener(name, callback);
  }

  function bind(obj, func) {
    return function() {
      return func.apply(obj, arguments);
    };
  }

  function vec2(x, y) {
    return { x: x, y: y };
  }

  function offsetOf(elem) {
    var p = vec2(0, 0);
    while (elem) {
      p.x += elem.offsetLeft;
      p.y += elem.offsetTop;
      elem = elem.offsetParent;
    }
    return p;
  }

  function relativeMouse(e, graph) {
    var offset = offsetOf(graph.element);
    return vec2(
      e.pageX - offset.x + graph.element.scrollLeft,
      e.pageY - offset.y + graph.element.scrollTop
    );
  }

  function nodeRect(node) {
    return {
      x: node.element.offsetLeft,
      y: node.element.offsetTop,
      width: node.element.clientWidth,
      height: node.element.clientHeight
    };
  }

  function connectionSite(connection, graph, isOutput) {
    var offset = offsetOf(connection.element);
    var graphOffset = offsetOf(graph.element);
    return vec2(
      offset.x - graphOffset.x + connection.element.clientWidth * isOutput,
      offset.y - graphOffset.y + connection.element.clientHeight / 2
    );
  }

  function removeFromList(items, item) {
    var index = items.indexOf(item);
    if (index != -1) items.splice(index, 1);
  }

  function drawLink(c, ax, ay, bx, by) {
    // Set up draw styles
    c.save();
    c.strokeStyle = 'yellow';
    c.fillStyle = 'yellow';
    c.lineWidth = 2;
    c.shadowBlur = 3;
    c.shadowColor = 'black';
    c.shadowOffsetY = 1;

    // Draw arrow curve
    c.beginPath();
    c.moveTo(ax, ay);
    c.bezierCurveTo(ax + 100, ay, bx - 90, by, bx, by);
    c.stroke();

    // Draw arrow head
    var t = 0.95, invT = 1 - t;
    var t0 = invT * invT * invT;
    var t1 = invT * invT * t * 3;
    var t2 = invT * t * t * 3;
    var t3 = t * t * t;
    var x = t0 * ax + t1 * (ax + 100) + t2 * (bx - 100) + t3 * bx;
    var y = t0 * ay + t1 * ay + t2 * by + t3 * by;
    var angle = Math.atan2(by - y, bx - x);
    var sin = Math.sin(angle);
    var cos = Math.cos(angle);
    c.beginPath();
    c.moveTo(bx, by);
    c.lineTo(bx - 10 * cos - 5 * sin, by - 10 * sin + 5 * cos);
    c.lineTo(bx - 10 * cos + 5 * sin, by - 10 * sin - 5 * cos);
    c.fill();
    c.restore();
  }

  var padding = 20;

  //////////////////////////////////////////////////////////////////////////////
  // class SelectionTool
  //////////////////////////////////////////////////////////////////////////////

  function SelectionTool(graph) {
    this.graph = graph;
    this.start = vec2(0, 0);
    this.end = vec2(0, 0);
    this.selecting = false;
  }

  SelectionTool.prototype = {
    onmousedown: function(e) {
      e.stopPropagation();
      e.preventDefault();
      this.selecting = true;
      this.start = relativeMouse(e, this.graph);
      this.onmousemove(e);
    },

    onmousemove: function(e) {
      if (this.selecting) {
        this.end = relativeMouse(e, this.graph);
        this.graph.draw();

        // Update the selected status of all nodes
        var start = this.start, end = this.end;
        var min = vec2(Math.min(start.x, end.x), Math.min(start.y, end.y));
        var max = vec2(Math.max(start.x, end.x), Math.max(start.y, end.y));
        for (var i = 0; i < this.graph.nodes.length; i++) {
          var rect = nodeRect(this.graph.nodes[i]);
          var selected = (max.x > rect.x && min.x < rect.x + rect.width &&
                          max.y > rect.y && min.y < rect.y + rect.height);
          this.graph.nodes[i].setSelected(selected);
        }

        // Update the selection box
        var c = this.graph.context;
        c.fillStyle = 'rgba(255, 255, 255, 0.1)';
        c.strokeStyle = 'rgba(255, 255, 255, 0.3)';
        c.fillRect(min.x, min.y, max.x - min.x, max.y - min.y);
        c.strokeRect(min.x + 0.5, min.y + 0.5, max.x - min.x, max.y - min.y);
      }
    },

    onmouseup: function(e) {
      this.selecting = false;
      this.graph.draw();
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class DraggingTool
  //////////////////////////////////////////////////////////////////////////////

  function DraggingTool(graph) {
    this.graph = graph;
    this.start = vec2(0, 0);
    this.prev = vec2(0, 0);
    this.min = vec2(0, 0);
    this.dragging = false;
  }

  DraggingTool.prototype = {
    onmousedown: function(e, node) {
      e.stopPropagation();
      e.preventDefault();
      this.dragging = true;
      this.prev = vec2(0, 0);
      this.start = relativeMouse(e, this.graph);
      if (!node.selected) this.graph.setSelection([node]);

      // Calculate the minimum allowed mouse coordinate so the nodes don't get dragged off the top left
      var min = vec2(Number.MAX_VALUE, Number.MAX_VALUE);
      this.graph.selection().map(function(node) {
        var rect = nodeRect(node);
        min.x = Math.min(min.x, rect.x);
        min.y = Math.min(min.y, rect.y);
      });
      this.min = min;
    },

    onmousemove: function(e) {
      if (this.dragging) {
        var mouse = relativeMouse(e, this.graph), prev = this.prev;
        var deltaX = Math.max(padding - this.min.x, mouse.x - this.start.x);
        var deltaY = Math.max(padding - this.min.y, mouse.y - this.start.y);
        this.graph.selection().map(function(node) {
          var rect = nodeRect(node);
          node.moveTo(deltaX + rect.x - prev.x, deltaY + rect.y - prev.y, true);
        });
        this.graph.updateBounds();
        this.prev = vec2(deltaX, deltaY);
      }
    },

    onmouseup: function(e) {
      this.dragging = false;
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class NodeLinkTool
  //////////////////////////////////////////////////////////////////////////////

  function NodeLinkTool(graph) {
    this.graph = graph;
    this.input = null;
    this.output = null;
    this.linking = false;
  }

  NodeLinkTool.prototype = {
    onmousedown: function(e, connection, isOutput) {
      e.stopPropagation();
      e.preventDefault();
      if (!isOutput && connection.targets.length) {
        this.output = connection.targets[connection.targets.length - 1];
        connection.disconnect(this.output);
      } else {
        this.output = isOutput ? connection : null;
      }
      this.linking = !!this.output;
      this.onmousemove(e);
    },

    onmousemove: function(e) {
      // Draw the temporary link on the graph canvas
      if (this.linking) {
        this.graph.draw();
        var mouse = relativeMouse(e, this.graph);
        var start = connectionSite(this.output, this.graph, true);
        var end = this.input ? connectionSite(this.input, this.graph, false) : mouse;
        drawLink(this.graph.context, start.x, start.y, end.x, end.y);
      }
    },

    onmouseup: function(e) {
      // Add a connection if the user made one
      if (this.linking && this.input && this.output) this.input.connect(this.output);
      this.linking = false;
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class Graph
  //////////////////////////////////////////////////////////////////////////////

  function Graph() {
    this.nodes = [];
    this.element = elem('div', null, 'GraphBox');
    this.canvas = elem('canvas', this.element);
    this.context = this.canvas.getContext('2d');
    this.selectionTool = new SelectionTool(this);
    this.draggingTool = new DraggingTool(this);
    this.nodeLinkTool = new NodeLinkTool(this);

    // Forward mouse events to tools
    on(this.canvas, 'mousedown', bind(this.selectionTool, this.selectionTool.onmousedown));
    on(document, 'mousemove', bind(this.selectionTool, this.selectionTool.onmousemove));
    on(document, 'mousemove', bind(this.draggingTool, this.draggingTool.onmousemove));
    on(document, 'mousemove', bind(this.nodeLinkTool, this.nodeLinkTool.onmousemove));
    on(document, 'mouseup', bind(this.selectionTool, this.selectionTool.onmouseup));
    on(document, 'mouseup', bind(this.draggingTool, this.draggingTool.onmouseup));
    on(document, 'mouseup', bind(this.nodeLinkTool, this.nodeLinkTool.onmouseup));
  }

  Graph.prototype = {
    draw: function() {
      // Draw the background
      var c = this.context;
      c.fillStyle = '#444';
      c.fillRect(0, 0, this.canvas.width, this.canvas.height);

      // Draw the grid
      c.strokeStyle = '#555';
      c.beginPath();
      for (var x = 0.5; x < this.canvas.width; x += 64) {
        c.moveTo(x, 0);
        c.lineTo(x, this.canvas.height);
      }
      for (var y = 0.5; y < this.canvas.height; y += 64) {
        c.moveTo(0, y);
        c.lineTo(this.canvas.width, y);
      }
      c.stroke();

      // Draw links from inputs to outputs
      for (var i = 0; i < this.nodes.length; i++) {
        var node = this.nodes[i];
        for (var j = 0; j < node.inputs.length; j++) {
          var input = node.inputs[j];
          var to = connectionSite(input, this, false);
          for (var k = 0; k < input.targets.length; k++) {
            var from = connectionSite(input.targets[k], this, true);
            drawLink(c, from.x, from.y, to.x, to.y);
          }
        }
      }
    },

    updateBounds: function() {
      // Resize the canvas to fit the contents
      var width = this.element.clientWidth;
      var height = this.element.clientHeight;
      for (var i = 0; i < this.nodes.length; i++) {
        var rect = nodeRect(this.nodes[i]);
        width = Math.max(width, rect.x + rect.width + padding);
        height = Math.max(height, rect.y + rect.height + padding);
      }

      // Browsers don't optimize resizing to the same size
      if (this.canvas.width != width || this.canvas.height != height) {
        this.canvas.width = width;
        this.canvas.height = height;
      }

      // Redraw the canvas since this method is called after moving things around
      this.draw();
    },

    setSelection: function(nodes) {
      for (var i = 0; i < this.nodes.length; i++) {
        this.nodes[i].setSelected(nodes.indexOf(this.nodes[i]) != -1);
      }
    },

    selection: function() {
      return this.nodes.filter(function(node) { return node.selected; });
    },

    addNode: function(node) {
      this.removeNode(node);
      node.updateHTML();
      this.nodes.push(node);
      this.element.appendChild(node.element);
      node.graph = this;
    },

    removeNode: function(node) {
      removeFromList(this.nodes, node);
      if (this.element == node.element.parentElement) {
        this.element.removeChild(node.element);
      }
      node.graph = null;
    },

    clear: function() {
      while (this.nodes.length) {
        this.removeNode(this.nodes[0]);
      }
      this.updateBounds();
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class Connection
  //////////////////////////////////////////////////////////////////////////////

  function Connection(name) {
    this.name = name || '';
    this.node = null;
    this.targets = [];
    this.element = null;
  }

  Connection.prototype = {
    connect: function(other) {
      this.disconnect(other);
      this.targets.push(other);
      other.targets.push(this);
      if (this.node && this.node.graph) this.node.graph.draw();
    },

    disconnect: function(other) {
      removeFromList(this.targets, other);
      removeFromList(other.targets, this);
      if (this.node && this.node.graph) this.node.graph.draw();
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class Node
  //////////////////////////////////////////////////////////////////////////////

  function Node(name) {
    this.graph = null;
    this.selected = false;
    this.name = name || '';
    this.inputs = [];
    this.outputs = [];
    this.element = elem('div', null, 'node');
    this.updateHTML();

    // Forward mouse events to the dragging tool
    on(this.element, 'mousedown', bind(this, function(e) {
      this.graph.draggingTool.onmousedown(e, this);
    }));
  }

  Node.prototype = {
    moveTo: function(x, y, /*optional*/ preventUpdate) {
      this.element.style.left = x + 'px';
      this.element.style.top = y + 'px';
      if (this.graph && !preventUpdate) this.graph.updateBounds();
    },

    input: function(name) {
      for (var i = 0; i < this.inputs.length; i++) {
        if (this.inputs[i].name == name) return this.inputs[i];
      }
      return null;
    },

    output: function(name) {
      for (var i = 0; i < this.outputs.length; i++) {
        if (this.outputs[i].name == name) return this.outputs[i];
      }
      return null;
    },

    setSelected: function(selected) {
      this.element.className = selected ? 'selected node' : 'node';
      this.selected = selected;
    },

    updateHTML: function() {
      // Reset content to title
      while (this.element.firstChild) {
        this.element.removeChild(this.element.firstChild);
      }
      elem('div', this.element, 'title').textContent = this.name;

      // Add inputs
      var cell = elem('td', elem('tr', elem('table', this.element)));
      for (var i = 0; i < this.inputs.length; i++) {
        var input = this.inputs[i];
        input.node = this;
        input.element = elem('div', cell, 'input');
        elem('div', elem('div', input.element, 'bullet'));
        text(input.name, input.element);
        (function(input) {
          on(input.element, 'mousedown', function(e) {
            input.node.graph.nodeLinkTool.onmousedown(e, input, false);
          });
          on(input.element, 'mouseover', function(e) {
            input.node.graph.nodeLinkTool.input = input;
          });
          on(input.element, 'mouseout', function(e) {
            input.node.graph.nodeLinkTool.input = null;
          });
        })(input);
      }

      // Add outputs
      cell = elem('td', cell.parentNode);
      for (var i = 0; i < this.outputs.length; i++) {
        var output = this.outputs[i];
        output.node = this;
        output.element = elem('div', cell, 'output');
        text(output.name, output.element);
        elem('div', elem('div', output.element, 'bullet'));
        (function(output) {
          on(output.element, 'mousedown', function(e) {
            output.node.graph.nodeLinkTool.onmousedown(e, output, true);
          });
        })(output);
      }

      // The bounds might have changed due to adding/removing inputs/outputs
      if (this.graph) this.graph.updateBounds();
    }
  };

  return { Graph: Graph, Node: Node, Connection: Connection };
})();
