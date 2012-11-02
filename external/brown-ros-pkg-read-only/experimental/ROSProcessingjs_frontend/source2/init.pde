void setup()
{
  size(500,300);
  background(125);
  fill(255);
  PFont fontA = loadFont("courier");
  textFont(fontA, 20);
  text("Welcome!", 20,20);
  noLoop();

  isRunning = false;
}

void draw()
{

}

void log(msg)
{
  println(msg);
}

