dojo.provide("misc.Loader");

dojo.declare("misc.Loader",null, {});

var loadVisualizationModule = function(filename,modulename) {
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl(modulename, filename), false);
	xhr.send('');
	var script = document.createElement('script');
	script.type = "text/javascript";
	script.text = xhr.responseText;
	document.getElementsByTagName('head')[0].appendChild(script);
}

var loadSource = function(filename,modulename)
{
	var xhr = new XMLHttpRequest();
	xhr.open('GET', dojo.moduleUrl(modulename, filename), false);
	xhr.send('');

  return xhr.responseText;
}

var loadCSS = function(filename,modulename)
{
  //Load common CSS
  var link = document.createElement('link');
  link.rel = "stylesheet";
  link.href = dojo.moduleUrl(modulename,filename);
  document.body.appendChild(link);
}

misc.Loader.loadSource = loadSource;
misc.Loader.loadVisualizationModule = loadVisualizationModule;
misc.Loader.loadCSS = loadCSS;
