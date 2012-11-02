String ip = "ws://localhost:9090";
String image_topic = "/robot/image_raw";
String image_uri = "http://localhost:8080/?topic=" + image_topic; 
PImage img;
int imageWidth = 480;
int imageHeight = 360;

boolean isRunning = false;
int mainLoopVar;
float resizeRate = 0.75;

PFont fontA = loadFont("courier");

boolean lock;

TagSeeking ts;

void setup()
{
  size(500,300);
  background(200);
  fill(10);
  textFont(fontA, 10);
  text("Nothing loaded yet!", 20,20);
  noLoop();

  connect(ip);
  isRunning = false;

  img = createImage(imageWidth,imageHeight,RGB);
  img = loadImage(image_uri);
  size(imageWidth,imageHeight);
  frameRate(50);

  noFill();
  stroke(0,255,0);
  strokeWeight(3);
}

void draw()
{
  image(img,0,0,imageWidth,imageHeight);

  if(isRunning)
  {
    Iterator i = ts.objList.entrySet().iterator();

    while(i.hasNext()) {
      Map.Entry me = (Map.Entry)i.next();
      ArTag t = me.getValue();
      float[] c= t.corners;
      quad(c[0],c[1],c[2],c[3],c[4],c[5],c[6],c[7]);
      //textFont(fontA,25);
      //text(""+t.id,(c[0]+c[4])/2,(c[1]+c[5])/2);
    }
  }
}

void run()
{
  println("Start");

  /* initialize */
  int[] pList = new int[2];
  pList[0] = 0;
  pList[1] = 6;
  
  ts = new TagSeeking(2,pList);

  subscribe('/robot/tags',processAR);
//  subscribe('/robot/image_raw',getCamStream);
  size(imageWidth,imageHeight);
  
  frameRate(30);
  loop();
  isRunning = true;
  mainLoopVar = setInterval(ts.process,1000);
}

void stop()
{
  move(0,0);
  println("Stop");

  noLoop();
  isRunning = false;
  clearInterval(mainLoopVar);
}

void getCamStream(msg)
{
  if(lock)
    return;
  
  lock = true;
  img = loadImage(msg.uri);  
  lock = false;
}

void processAR(tags)
{
  int i;
  float ctr;
  ArTag newtag;
  
  ts.objList.clear();
  
  for(i = 0; i < tags.tag_count;i++) {
    newtag = new ArTag(tags.tags[i].id,tags.tags[i].x,tags.tags[i].distance,tags.tags[i].diameter,tags.tags[i].cwCorners);

    ts.objList.put(tags.tags[i].id,newtag);
  }
}

class TagSeeking
{
  float previous_error = 0;
  float kp = 0.001;
  float kd = 0.002;
  int center = 320;

  HashMap objList;
  int paramLength;
  int paramList;
  int index;
  int objNum;
  
  boolean seekStatus;
  boolean nextStatus;

  TagSeeking(int pLength,int[] pList)
  {
    paramLength = pLength;
    paramList = pList;
    
    init();
  }

  void init()
  {
    seekStatus = false;
    nextStatus = false;
    index = 0;
    objNum = paramList[index];
    objList = new HashMap();
  }

  void process()
  {
    if(index < paramLength)
    {
      // check if it lose the target
      if(objList.containsKey(objNum)) 
        seekStatus = true;
      else
        seekStatus = false;

      println("seekStatus = " +seekStatus);
      
      // move the robot
      if(seekStatus)
      {
        nextStatus = approachToObject(objList.get(objNum));
      }
      else {
        move(0,0.3);
      }

      // if it approached enough to the target, then switch the target to next one:w
      if(nextStatus) {
        move(0,0);
        index++;
        

        if(index < paramLength)
          objNum = paramList[index];

        nextStatus = false;
        println("next = " + objNum);
      }
    }
    else {
      println("Done!!");
    }
  }

  boolean approachToObject(obj)
  {
    float va;
    if(obj != null)
    {
      if(obj.diameter < 200) {
        va = computeAngularVelocity(obj.x);
        move(0.1,va);
      }
      else {
        println("found it...");
        return true;
      }
    }
    return false;
  }

  float computeAngularVelocity(x)
  {
    float error;
    float derivative;
    float va;
  
    error = center - x;
    derivative= error - previous_error;
    va = kp * error + kd * derivative;
    previous_error = error;

    return va;
  }
}


class ArTag {
  int id;
  int x;
  int distance;
  float diameter;
  float[] corners;

  ArTag(int i,int _x,int dist,float dmeter,float[] c)
  {
    id = i;
    x = _x;
    distance = dist;
    diameter = dmeter;
    corners = new float[8];

    for(int i =0; i < 8; i++)
    {
      corners[i] = c[i] * resizeRate;
    }
  }
}

