$(function() {
	init();
});

//Added by tjay (loc sync)
var connect = function(host, post) {};
var report = function() {};


// MAP VARS
var pi=Math.PI;

var map_x_windowloc = 0;
var map_y_windowloc = 0;
var map_x_windowsize = 480;
var map_y_windowsize = 480;

var playerPos=[map_x_windowsize/2,map_y_windowsize/2]; // x,y (from top left)
var playerDir=0; // theta, facing right=0=2pi

playerViewAngle = Math.PI/3; // 60 degrees, in radians
playerViewDistance = 20;     // 20 pixels.. for now.. 

// End MAP VARS

var map;

function init()
{
  map_x_windowloc = getElLeft(document.getElementById('map'));
  map_y_windowloc = getElTop(document.getElementById('map'));   
   
  map = document.getElementById('map').getContext('2d');

  var applet = $("#javaBlobs")[0];

  connect = function(host, port) {
    applet.connect(host,port);
  }

  report = function() {
    return applet.report();
  }

  setInterval(update,30);
}
 
function update()
{          
   position = eval(report());

   playerPos[0] = (position[0]*120);
   playerPos[1] = 480 - (position[1]*120);
   playerDir = -position[2];

   
   drawMap();
}

///////////////////////////////////////////////////////
// Utility code
///////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////
// Map code
///////////////////////////////////////////////////////

function drawMap()
{

   // Begin
   map.clearRect(0,0,map_x_windowsize,map_y_windowsize);
 
   // Draw player
   map.beginPath();
   map.fillStyle="#0000ff";
   map.arc(playerPos[0], playerPos[1], 3, 0, 2*pi, true);
   map.closePath();
   map.fill();
   map.beginPath();
   map.moveTo(playerPos[0], playerPos[1]);

   theta1=playerDir+playerViewAngle/2;
   theta2=playerDir-playerViewAngle/2;
   var point1 = [playerPos[0] + playerViewDistance*Math.cos(theta1), playerPos[1] + playerViewDistance*Math.sin(theta1)];
   var point2 = [playerPos[0] + playerViewDistance*Math.cos(theta2), playerPos[1] + playerViewDistance*Math.sin(theta2)];

   map.arc(playerPos[0], playerPos[1], playerViewDistance, playerDir+playerViewAngle/2, playerDir-playerViewAngle/2, true);

   map.fillStyle="#FFFFFF"
   map.fill();
   map.closePath();
}

function calculateXVal(direction, magnitude)
{
   return (magnitude*Math.cos(direction));
}

function calculateYVal(direction, magnitude)
{ 
   return (magnitude*Math.sin(direction));
}
