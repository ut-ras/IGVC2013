int mainLoopVal;
PFont fontA = loadFont("courier");

boolean bumpLeft;
boolean bumpRight;

void mySetup()
{
  bumpLeft = false;
  bumpRight = false;
}

void myDraw()
{
  if(bumpLeft || bumpRight) {
    text("hello create, you have bumped", 20,20);
  }
  else {
    text("hello create, you can spin", 20,20);
  }
}

void myRun()
{
  println("Start");
  subscribe('/sensorPacket',processSensing);
  mainLoopVal = setInterval(enclosure_escape,500);
}

void myStop()
{
  println("Stop");
  move(0,0);
  clearInterval(mainLoopVal);
}

void move_robot(x,z) {
  publish('/cmd_vel','geometry_msgs/Twist','{"linear":{"x":' + x + ',"y":0,"z":0}, "angular":{"x":0,"y":0,"z":' + z + '}}');
}

void processSensing(sensorPacket)
{
  bumpLeft = sensorPacket.bumpLeft;
  bumpRight = sensorPacket.bumpRight;
}

void enclosure_escape()
{
  if(bumpLeft || bumpRight)
  {
    println("hello create, you have bumped into something");
    x = -0.1;
    z = 0.8;
//    background(255,0,0,100);
//    text("hello create, you have bumped", 20,20);
  }
  else
  { 
    println("hello create, you can spin now");
    x = 3;
    z = -10;
//    text("hello create, you can spin", 20,20);
  }
  move_robot(x,z);
}

