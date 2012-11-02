
PFont fontA = loadFont("courier");

void mySetup()
{
  textFont(fontA,20);
}

void myDraw()
{
  text("Wait Until the robot is docked",20,20);
}

void myRun()
{
  println("Start");
  ros.callService('/dock','[]',nop);
}

void myStop()
{
}
