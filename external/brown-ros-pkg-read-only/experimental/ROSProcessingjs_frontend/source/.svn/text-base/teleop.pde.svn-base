int mainLoopVal;
PFont fontA = loadFont("courier");

float x_vel;
float z_vel;

void mySetup()
{
  background(200);
  fill(10);
  textFont(fontA, 20);
  text("Move up    : W",20,20);
  text("Move down  : S",20,35);
  text("Turn left  : A",20,50);
  text("Turn right : D",20,65);  

  x_vel = 0.4;
  z_vel = 1;
}
  
void myDraw()
{
  text("Move up    : W",20,20);
  text("Move down  : S",20,35);
  text("Turn left  : A",20,50);
  text("Turn right : D",20,65);  
}

void myRun()
{
  println("Start");
//  mainLoopVar = setInterval(mainLoop,1000);
}

void myStop()
{
  println("Stop");
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
