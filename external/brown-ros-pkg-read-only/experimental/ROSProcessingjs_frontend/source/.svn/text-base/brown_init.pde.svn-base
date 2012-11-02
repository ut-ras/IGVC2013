/* Setting Connection */
String ip = "138.16.160.10"; 
String wsip = "ws://" + ip + ":9090";

/* Setting for MJPEG Image Stream */
PImage _camImage;
String camtopic = "/overhead/image_raw";
String _camuri = "http://"+ip+":8080/?topic=" + camtopic; 
int camWidth = 480;
int camHeight = 360;

boolean isRunning = false;
PFont _initFont = loadFont("courier");

void setup()
{
  size(500,300);
  background(125);
  fill(255);
  textFont(_initFont, 20);
  text("New Source Initialized!", 20,20);
  
  isRunning = false;
  ip = "138.16.160.10";

  mySetup();
  
  wsip = "ws://" + ip + ":9090";

  _camuri = "http://"+ip+":8080/?topic=" + camtopic; 
  setCamSize(camWidth,camHeight);

  loop();
  println(wsip);
  connect(wsip);
}

void draw()
{
  if(camtopic)
    image(_camImage, 0, 0, camWidth, camHeight);
  if(isRunning) {
    myDraw();
  }
}

void run()
{
  isRunning = true;
  myRun();
}

void stop()
{
  myStop();
  isRunning = false;
}

void setCamSize(int cw,int ch) { 
  size(cw,ch);
  camWidth = cw;
  camHeight = ch;
  
  _camuri = "http://"+ip+":8080/?topic=" + camtopic; 
//  _camuri = _camuri + "?width=" + camWidth + "?height=" + camHeight;
  
  _camImage = createImage(camWidth,camHeight, RGB);
  _camImage = loadImage(_camuri);
  frameRate(20);
}

