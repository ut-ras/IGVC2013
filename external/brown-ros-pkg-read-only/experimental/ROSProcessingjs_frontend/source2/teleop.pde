String ip = "ws://localhost:9090";
String image_topic = "image_topic";
String image_uri = "http://localhost:8080/?topic=" + image_topic; 
PImage img;
int imageWidth = 480;
int imageHeight = 360;

boolean isRunning = false;
int mainLoopVar;

PFont fontA = loadFont("courier");

float x_vel;
float z_vel;

void setup()
{
  size(500, 300);
  background(200);
  fill(10);
  textFont(fontA, 20);
  text("Move up   : W",20,20);
  text("Move down : S",20,35);
  text("Turn left : A",20,50);
  text("Turn right: D",20,65);  
  
  connect(ip);
  isRunning = false;

  img = createImage(imageWidth,imageHeight, RGB);
  img = loadImage(image_uri);
  size(imageWidth,imageHeight);
  frameRate(50);
  loop();

  x_vel = 0.4;
  z_vel = 1;

}
  
void draw()
{
  image(img, 0, 0, imageWidth, imageHeight);
  if(isRunning) {
    // do something
  text("Move up   : W",20,20);
  text("Move down : S",20,35);
  text("Turn left : A",20,50);
  text("Turn right: D",20,65);  

  }
}

void run()
{
  println("Start");
  isRunning = true;
//  mainLoopVar = setInterval(mainLoop,1000);
}

void stop()
{
  println("Stop");
  isRunning = false;
//  clearInterval(mainLoopVar);
}


void keyPressed()
{
  if(isRunning) {
    if(key=='w' || key=='W') { 
      move(x_vel, 0);
    }
    else if(key=='s' || key=='S') { 
      move(-x_vel, 0);
    }
    else if(key=='a' || key=='A') {
      move(0, z_vel);
    }  
    else if(key=='d' || key=='D') {
      move(0, -z_vel);
    }
  }
}

void keyReleased()
{
  move(0, 0);
}
