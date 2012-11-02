//Load common CSS
var link = document.createElement('link');
link.rel = "stylesheet";
link.href = dojo.moduleUrl("museum", "css/museum.css");
document.body.appendChild(link);

var link2 = document.createElement('link');
link2.rel = "stylesheet";
link2.href = dojo.moduleUrl("museum", "css/buttons.css");
document.body.appendChild(link2);
