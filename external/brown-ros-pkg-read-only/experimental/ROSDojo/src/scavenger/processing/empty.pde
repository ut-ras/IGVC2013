PImage img;
int imageWidth = 480;
int imageHeight = 360;
int x;
int y;

//String image_topic = "/kinect_head/rgb/image_color";
String image_topic = "/wide_stereo/right/image_color";

String image_uri = "http://pro:8080/?topic=" + image_topic+"?width="+imageWidth+"?height="+imageHeight; 
PFont fontA = loadFont("courier");

boolean isPressed = false;

void setup()
{
  size(500,300);
  background(125);
  fill(255);
  textFont(fontA, 20);
  text("Nothing loaded yet!", 20,20);

  frameRate(50);
  loop();

  img = createImage(imageWidth,imageHeight,RGB);
  img = loadImage(image_uri);
  size(imageWidth,imageHeight);
  frameRate(50);

  fill(255,255,255);
}


void draw()
{
  image(img,0,0,imageWidth,imageHeight);
//  background(125);
  if(isPressed)
    arc(x,y,25,25,0,2 * PI);

}

void mousePressed()
{
  isPressed = true;
  x = mouseX;
  y = mouseY;
}

void mouseDragged()
{
  x = mouseX;
  y = mouseY;
}

void mouseReleased()
{
  isPressed = false;
  x = mouseX;
  y = mouseY;
}
