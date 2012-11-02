var fwdVel = 0;
var turnVel = 0;

var lastFwdVel = -1;
var lastTurnVel = -1;

var scaleFactor = 1;

var joystick_x_windowloc = 0;
var joystick_y_windowloc = 0;

var joystick_x_windowsize = 300;
var joystick_y_windowsize = 300;

var joystick_center_x = 150;
var joystick_center_y = 150;

var joystick_x = 150;
var joystick_y = 150;

var mouseDown = false;

var connect;
var report;

function init()
{
  var javaJoystick = document.getElementById('javaJoystick');
  connect = function(host, port) {
	javaJoystick.connect(host,port);
  }
  report = function(x, z) {
  	z = 255 - z;
  	if (Math.abs(x - 127) < 15) {
		x = 127;
	}
	x -= 128;
	z -= 128;
	javaJoystick.report(x,z);
  }

  joystick_x_windowloc = getElLeft(document.getElementById('joystick'));
  joystick_y_windowloc = getElTop(document.getElementById('joystick'));    
  setInterval(update,5);

  // Find the canvas element.
  var canvas = document.getElementById('joystick');
  if (!canvas) 
  {
     alert('Error: I cannot find the canvas element!');
     return;
  }

  joystick()
}
 
function update()
{          
   fwdVel = Math.floor(((300-joystick_y)/300) * 255);
   turnVel= Math.floor((joystick_x/300) * 255);
 
   if(fwdVel>255)
   {
      fwdVel=255;
   }
   else if(fwdVel<0)
   {
      fwdVel=0;
   }
      
   if(turnVel>255)
   {
      turnVel=255;
   }
   else if(turnVel<0)
   {
      turnVel=0;
   }
   
   document.getElementById('fwdVel').value = 'fwdVel:' + fwdVel;
   document.getElementById('turnVel').value = 'turnVel:' + turnVel;
   
   if((fwdVel!=lastFwdVel) || (turnVel!=lastTurnVel))
   {
      report(fwdVel, turnVel);
      lastFwdVel = fwdVel;
      lastTurnVel = turnVel;
   }
   
   joystick();
}

///////////////////////////////////////////////////////
// Joystick code
///////////////////////////////////////////////////////

function joystick()
{
  var ctx = document.getElementById('joystick').getContext('2d');
  var circleDiameter = 25;

  ctx.save();
  ctx.clearRect(0,0,joystick_x_windowsize,joystick_y_windowsize);
  ctx.translate(joystick_x, joystick_y);
  ctx.scale(.4,.4);
  ctx.rotate(-Math.PI/2);
  ctx.strokeStyle = "black";
  ctx.fillStyle = "black";
  ctx.lineWidth = 8;
  ctx.lineCap = "round";

  // Draw circle
  ctx.beginPath();
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#325FA2';
  ctx.arc(0,0,circleDiameter,0,Math.PI*2,true);
  ctx.fill()
  
  ctx.restore();
}

function getMouseCoords(event)
{
   if(event == null)
   {
      event = window.event; 
   }
   if(event == null)
   {
      return null; 
   }
   if(event.pageX || event.pageY)
   {
      return {x:event.pageX, y:event.pageY};
   }
   return null;
}

/// Event Listeners..
document.onmousedown = function(event)
{
   var mouseCoords; 

   mouseCoords = getMouseCoords(event); 
   if(mouseCoords == null)
   {
      return; 
   }

   joystick_x = mouseCoords.x - joystick_x_windowloc;
   joystick_y = mouseCoords.y - joystick_y_windowloc;

   mouseDown = true;
}

document.onmouseup = function(event)
{
   mouseDown = false;

   joystick_x = joystick_center_x;
   joystick_y = joystick_center_y;
}

document.onmousemove = function(event)
{
   if(mouseDown)
   {
      var mouseCoords; 

      mouseCoords = getMouseCoords(event); 
      if(mouseCoords == null)
      {
         return; 
      }
      
      joystick_x = mouseCoords.x - joystick_x_windowloc;
      joystick_y = mouseCoords.y - joystick_y_windowloc;
   }
}

// This function used to get canvas' leftmost position
function getElLeft(el) 
{
   var xPos = el.offsetLeft;
   var tempEl = el.offsetParent;
   while (tempEl != null) 
   {
      xPos += tempEl.offsetLeft;
      tempEl = tempEl.offsetParent;
   }
   return xPos;
}

// This function used to get canvas' topmost position
function getElTop(el) 
{
 	var yPos = el.offsetTop;
   var tempEl = el.offsetParent;
   while (tempEl != null) 
   {
      yPos += tempEl.offsetTop;
      tempEl = tempEl.offsetParent;
   }
   return yPos;
}
