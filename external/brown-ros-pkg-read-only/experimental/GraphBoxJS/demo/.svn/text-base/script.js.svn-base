"use strict";

////////////////////////////////////////////////////////////////////////////////
// Graph
////////////////////////////////////////////////////////////////////////////////

function createGraph(nodeNames, state) {
  // Create nodes
  var nodes = {};
  nodeNames.map(function(name) {
    nodes[name] = new GraphBox.Node(name);
  });

  // Create outputs
  Object.keys(state.published_topics).map(function(topic) {
    state.published_topics[topic].map(function(name) {
      nodes[name].outputs.push(new GraphBox.Connection(topic));
    });
  });

  // Create inputs
  Object.keys(state.subscribed_topics).map(function(topic) {
    state.subscribed_topics[topic].map(function(name) {
      nodes[name].inputs.push(new GraphBox.Connection(topic));
    });
  });

  // Link up inputs to outputs
  Object.keys(state.subscribed_topics).map(function(topic) {
    if (topic in state.published_topics) {
      state.subscribed_topics[topic].map(function(name) {
        var input = nodes[name].input(topic);
        state.published_topics[topic].map(function(name) {
          input.connect(nodes[name].output(topic));
        });
      });
    }
  });

  // Update graph
  graph.clear();
  Object.keys(nodes).map(function(name) {
    graph.addNode(nodes[name]);
  });
  graph.updateBounds();
}

var graph = new GraphBox.Graph();
document.body.appendChild(graph.element);
graph.updateBounds();

////////////////////////////////////////////////////////////////////////////////
// Queue
////////////////////////////////////////////////////////////////////////////////

var queue = {
  tasks: [],

  clear: function() {
    this.tasks = [];
  },

  run: function() {
    if (!this.tasks.length) return;
    var task = this.tasks.shift();
    ROS.onmessage = function(response) {
      task.completed(response);
      ROS.onmessage = null;
      queue.run();
    };
    ROS.send(task.request);
  },

  task: function(request, completed) {
    this.tasks.push({
      request: request,
      completed: completed
    });
  }
};

function queryTopics() {
  var nodes = [];
  queue.clear();
  queue.task({ receiver: '/rosbridge/nodes', msg: [] }, function(data) {
    nodes = data.msg;
  });
  queue.task({ receiver: '/rosbridge/system_state', msg: [] }, function(data) {
    var systemState = data.msg;
    createGraph(nodes, systemState);
  });
  queue.run();
}

ROS.onopen = queryTopics;
ROS.connect();
