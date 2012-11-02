dojo.provide("roswidgets.Utils");

dojo.declare("roswidgets.Utils",null, {});

var loadVisualizationModule = function(filename) {
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl("roswidgets", filename), false);
	xhr.send('');
	var script = document.createElement('script');
	script.type = "text/javascript";
	script.text = xhr.responseText;
	document.getElementsByTagName('head')[0].appendChild(script);
}

roswidgets.Utils.loadVisualizationModule = loadVisualizationModule;
roswidgets.Utils.loadSource = function(filename,modulename)
{
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl(modulename, filename), false);
	xhr.send('');

  return xhr.responseText;
}

roswidgets.Utils.loadVisualizationModule("lib/processingjs/processing-1.0.0.min.js");
