String ip = "ws://localhost:9090";
String image_topic = "image_topic";
String image_uri = "http://localhost:8080/?topic=" + image_topic; 
PImage img;
int imageWidth = 480;
int imageHeight = 360;

boolean isRunning = false;
int mainLoopVar;

PFont fontA = loadFont("courier");

// Cliff Sensor Values
int l = 0;
int lf = 0;
int rf = 0;
int r = 0;

// Driving speed (negative rot: CW, positive rot: CCW) 
float fwdSpeed = 0.0;
float rotSpeed = 0.0;

boolean bumpLeft;
boolean bumpRight;

boolean lock;

void setup()
{
  size(500,300);
  background(125);
  fill(255);
  textFont(fontA, 20);
  text("Nothing loaded yet!", 20,20);

  connect(ip);
  isRunning = false;

  img = createImage(imageWidth,imageHeight, RGB);
  img = loadImage(image_uri);
  size(imageWidth,imageHeight);
  frameRate(50);
  loop();
}


void draw()
{
  image(img, 0, 0, imageWidth, imageHeight);
  if(isRunning) {
    // do something
  }
}

void run()
{
  println("Start");
  subscribe('/sensorPacket',processSensing);
  isRunning = true;
  mainLoopVar = setInterval(line_follow,3000);
}

void stop()
{
  println("Stop");
  move(0,0);
  isRunning = false;
  clearInterval(mainLoopVar);
}

void line_follow()
{
  int lCliffValue = 0;
  int lfCliffValue = 0;
  int rfCliffValue = 0;
  int rCliffValue = 0;
  
  // 0: no Detected, 1: lSensor, 2: lfSensor, 3: rfSensor, 4: rSensor
  int onTheLine = 0;

  lock = true;
  lCliffValue = l;
  lfCliffValue = lf;
  rfCliffValue = rf;
  rCliffValue = r;
  lock = false;

	if(lCliffValue - rCliffValue > 300)
	{
		onTheLine = 1;
		fwdSpeed = 0.0;
		rotSpeed = 0.8;
	}
	else if(lfCliffValue - rfCliffValue > 150)
	{
		onTheLine = 2;
		fwdSpeed = 0.0;
		rotSpeed = 0.6;		
	}
	else if(rfCliffValue - lfCliffValue > 450)
	{
		onTheLine = 3;
		fwdSpeed = 0.0;
		rotSpeed = -0.6;
	}
	else if(rCliffValue - lCliffValue> 400)
	{
		onTheLine = 4;
		fwdSpeed = 0.0;
		rotSpeed = -0.8;
	}
	else
	{
		onTheLine = 0;
		fwdSpeed = 0.08;
		rotSpeed = 0.0;
	}

  move(fwdSpeed,rotSpeed);

	println("Currently I'm on #" + onTheLine);
	println("** Sensor Value Detail **");
	println("left Sensor Level        = " + lCliffValue);
	println("front-left Sensor Level  = " + lfCliffValue);
	println("front-right Sensor Level = " + rfCliffValue);
	println("right Sensor Level       = " + rCliffValue);

}

void processSensing(msg)
{
  if(lock)
    return;
  l = msg.cliffLeftSignal;
  lf = msg.cliffFrontLeftSignal;
  rf = msg.cliffFrontRightSignal;
  r = msg.cliffRightSignal;

  bumpLeft = msg.bumpLeft;
  bumpRight = msg.bumpRight;
}
