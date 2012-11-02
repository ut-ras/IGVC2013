String ip = "ws://localhost:9090";
boolean isRunning = false;
int mainLoopVal;

PFont fontA = loadFont("courier");

PImage img;
int imageWidth = 640;
int imageHeight = 480;
int resizeRate = 0.5;
boolean lock;

void setup()
{
  size(500,300);
  background(125);
  fill(255);
  textFont(fontA, 20);
  text("Nothing loaded yet!", 20,20);
  noLoop();

  connect(ip);
  isRunning = false;

  imageWidth = imageWidth * resizeRate;
  imageHeight = imageHeight * resizeRate;
  img = createImage(imageWidth,imageHeight, RGB);
  lock = false;
}


void draw()
{
  if(isRunning) {
    image(img, 0, 0, imageWidth, imageHeight);
  }
}

void run()
{
  println("Start");
  subscribe('/gscam/image_raw',getCamStream);
  size(imageWidth,imageHeight);

  frameRate(50);
  loop();
  isRunning = true;
  //mainLoopVar = setInterval(mainLoop,1000);
}

void stop()
{
  println("Stop");

  isRunning = false;
  noLoop();
  //clearInterval(mainLoopVar);
}


void getCamStream(msg)
{
  if(lock)
    return;
  
  lock = true;
  img = loadImage(msg.uri);  
  lock = false;
}
