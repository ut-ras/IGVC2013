dojo.provide("userstudy.Utils");

var loadVisualizationModule = function(filename) {
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl("userstudy", filename), false);
	xhr.send('');
	var script = document.createElement('script');
	script.type = "text/javascript";
	script.text = xhr.responseText;
	document.getElementsByTagName('head')[0].appendChild(script);
}


dojo.declare("userstudy.Utils", null, {});

userstudy.Utils.loadVisualizationModule = loadVisualizationModule;
userstudy.Utils.loadSource = function(filename)
{
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl("userstudy", filename), false);
	xhr.send('');

  return xhr.responseText;
}

userstudy.Utils.loadVisualizationModule("lib/processingjs/processing-1.0.0.min.js");

