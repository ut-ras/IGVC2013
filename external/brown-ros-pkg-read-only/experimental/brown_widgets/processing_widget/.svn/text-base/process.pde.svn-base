
void setup() {
  size(400,400);
  background(125);
  fill(255);

  PFont fontA = loadFont("courier");
  textFont(fontA, 20);

  t = 0;
  noLoop();

  posx = width/2;
  posy = height/2;

}


void draw()
{
  background(125);
  fill(255);

  rect(posx-25,posy-25,50,50);
}

void updateCanvas(msg)
{
  posx += -msg.angular.z * 5;
  posy += -msg.linear.x * 5;
 redraw();
}

void log(msg)
{
  println(msg);
}

