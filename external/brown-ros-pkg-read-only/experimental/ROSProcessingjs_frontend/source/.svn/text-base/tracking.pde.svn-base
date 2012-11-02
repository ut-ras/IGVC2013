
int mainLoopVal;
PFont fontA = loadFont("courier");

float x_vel;
float z_vel;
float x=0;
float y=0;

void mySetup()
{
  camWidth = 640;
  camHeight = 480;
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

    stroke(255,0,0);
    noFill();
    arc(x,y,100,100,0,TWO_PI);

}

void myRun()
{
    println("Start");
    subscribe('/blobgroup',processBlob);
    //  mainLoopVar = setInterval(mainLoop,1000);
}

void processBlob(msg)
{
  x = msg.pose[0].position.x;
  y = msg.pose[0].position.y;
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

