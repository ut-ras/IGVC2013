boolean flag;
float start_mousePosX;
float start_mousePosY;
float end_mousePosX;
float end_mousePosY;
PImage img;

void setup() {
  size(100,100);
  PFont fontA = loadFont("courier");
  textFont(fontA, 100);

  t = 0;

  posx = width/2;
  posy = height/2;

  flag = false;

  img = createImage(320,240);

  start_mousePosX=start_mousePosY = end_mousePosX =end_mousePosY =0;

  strokeWeight(5);
  fill(0,255,0,50);
  stroke(0,255,0);
  loop();
}

void draw()
{
  //background(200);
  size(img.width,img.height);
  image(img,0,0);

  rect(start_mousePosX,start_mousePosY,end_mousePosX-start_mousePosX,end_mousePosY - start_mousePosY);

}

void resetImage(img_topic)
{
  PImage new_img = createImage(img_topic.width,img_topic.height);
  
  imgPixels = window.atob(img_topic.data);

  for(int i=0; i < new_img.width * new_img.height; i++)
  {
    color c = color(imgPixels.charCodeAt(i*3),imgPixels.charCodeAt(i*3+1),imgPixels.charCodeAt(i*3+2));
    new_img.pixels[i] = c;
  }

  start_mousePosX=start_mousePosY = end_mousePosX =end_mousePosY =0;

  img = new_img;
}

void setStream(uri,topic)
{
  String camuri ="http://"+uri+":8080/?topic="+topic;

  size(800,600);
  PImage wowimg = createImage(800,600,RGB);
  wowimg =loadImage(camuri);
  img=wowimg;
  frameRate(20);
}

void mousePressed()
{
  start_mousePosX = mouseX;
  start_mousePosY = mouseY;
  end_mousePosX = mouseX;
  end_mousePosY = mouseY;
}

void mouseDragged()
{
  end_mousePosX = mouseX;
  end_mousePosY = mouseY;
}

void mouseReleased()
{
  end_mousePosX = mouseX;
  end_mousePosY = mouseY;
}

float getStartPosX() { return start_mousePosX; }
float getStartPosY() { return start_mousePosY; }
float getEndPosX() { return end_mousePosX; }
float getEndPosY() { return end_mousePosY; }
