String ip = "ws://localhost:9090";
String image_topic = "image_topic";
String image_uri = "http://localhost:8080/?topic=" + image_topic; 
PImage img;
int imageWidth = 480;
int imageHeight = 360;

boolean isRunning = false;
int mainLoopVal;

PFont fontA = loadFont("courier");

void setup()
{
  size(500,300);
  background(125);
  fill(255);
  textFont(fontA, 20);
  text("Nothing loaded yet!", 20,20);

  connect(ip);
  isRunning = false;

  img = createImage(imageWidth,imageHeight, RGB);
  img = loadImage(image_uri);
  size(imageWidth,imageHeight);
  frameRate(50);
  loop();
}

void draw()
{
  image(img, 0, 0, imageWidth, imageHeight);
  if(isRunning) {
    // do something
  }
}

void run()
{
  println("Start");
  isRunning = true;
  //mainLoopVar = setInterval(mainLoop,1000);
}

void stop()
{
  println("Stop");
  isRunning = false;
  //clearInterval(mainLoopVar);
}
