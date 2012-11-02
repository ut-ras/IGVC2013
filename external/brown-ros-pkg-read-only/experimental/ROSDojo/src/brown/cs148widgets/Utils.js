// Load code mirror
var xhr = new XMLHttpRequest();
xhr.open('GET', dojo.moduleUrl("cs148widgets", "lib/lib/codemirror.js"), false);
xhr.send('');
var script = document.createElement('script');
script.type = "text/javascript";
script.text = xhr.responseText;
document.getElementsByTagName('head')[0].appendChild(script);

// Load python
xhr = new XMLHttpRequest();
xhr.open('GET', dojo.moduleUrl("cs148widgets", "lib/mode/python/python.js"), false);
xhr.send('');
script = document.createElement('script');
script.type = "text/javascript";
script.text = xhr.responseText;
document.getElementsByTagName('head')[0].appendChild(script);

//Load Skulpt
xhr = new XMLHttpRequest();
xhr.open('GET', dojo.moduleUrl("cs148widgets", "lib/skulpt.js"), false);
xhr.send('');
script = document.createElement('script');
script.type = "text/javascript";
script.text = xhr.responseText;
document.getElementsByTagName('head')[0].appendChild(script);

//Load Skulpt builtins
xhr = new XMLHttpRequest();
xhr.open('GET', dojo.moduleUrl("cs148widgets", "lib/builtin.js"), false);
xhr.send('');
script = document.createElement('script');
script.type = "text/javascript";
script.text = xhr.responseText;
document.getElementsByTagName('head')[0].appendChild(script);

//Load narrative js
xhr = new XMLHttpRequest();
xhr.open('GET', dojo.moduleUrl("cs148widgets", "lib/njs_compile.js"), false);
xhr.send('');
script = document.createElement('script');
script.type = "text/javascript";
script.text = xhr.responseText;
document.getElementsByTagName('head')[0].appendChild(script);

// Load the code mirror CSS
var link = document.createElement('link');
link.rel = "stylesheet";
link.href = dojo.moduleUrl("cs148widgets", "lib/lib/codemirror.css");
document.body.appendChild(link);

//Load common CSS
var link = document.createElement('link');
link.rel = "stylesheet";
link.href = dojo.moduleUrl("cs148widgets", "css/cs148widgets.css");
document.body.appendChild(link);