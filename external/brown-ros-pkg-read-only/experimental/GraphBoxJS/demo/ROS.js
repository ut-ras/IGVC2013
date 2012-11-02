"use strict";

var ROS = {
  url: 'ws://' + location.hostname + ':9090',
  socket: null,

  disconnect: function() {
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  },

  connect: function() {
    var self = this;
    this.disconnect();
    this.socket = new WebSocket(this.url);
    this.socket.onopen = function() {
      if (self.onopen) self.onopen();
    };
    this.socket.onclose = function() {
      self.disconnect();
      if (self.onclose) self.onclose();
      setTimeout(function() { self.connect(); }, 500);
    };
    this.socket.onerror = function() {
      self.disconnect();
      if (self.onclose) self.onclose();
      setTimeout(function() { self.connect(); }, 500);
    };
    this.socket.onmessage = function(e) {
      if (self.onmessage) self.onmessage(JSON.parse(e.data));
    };
    this.send = function(json) {
      var data = JSON.stringify(json);
      if (self.socket) self.socket.send(data);
    };
  }
};
