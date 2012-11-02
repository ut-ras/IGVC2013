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

  frameRate(50);
  loop();
}


void draw()
{
}
