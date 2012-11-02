#include <dynamic_reconfigure/server.h>
#include <gests/gestsConfig.h>
#include "ges.hpp"
#include <fstream>
#include <signal.h>

#define WRITEDATA 1

image_transport::Publisher dbgPub;
image_transport::Publisher dptPub;
IplImage* dbgImg;
sensor_msgs::CvBridge bridge;

ros::Publisher posePub;

Gesture* g;
double cConst, cCameraY;
int cTolerance, cMinSize;
int cWidth, cHeight;
//char cTrainingFile[50];
string cTrainingFile;

#ifdef WRITEDATA
ofstream outputFile;
char debugMsg[50];

void getDebugMsg(const std_msgs::String::ConstPtr& msg) {
  //  debugMsg.copy(msg->data.c_str()); needs some kind of cast
  strcpy(debugMsg, msg->data.c_str());
}

#endif

void gParamCB(gests::gestsConfig &config, uint32_t level) {
  cConst = config.personConst;
  //  cOffset = config.personOffset;
  cTolerance = config.tolerance;
  cMinSize = config.minSize;
  cCameraY = config.cameraY;
}

void processPoints(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  g->person.setCalibConst(cConst);
  //  g->person.setCalibOffset(cOffset);
  g->person.setTolerance(cTolerance);
  g->person.setMinSize(cMinSize);
  g->person.setCameraY(cCameraY);

  g->checkOutCloud(cloud);

  sensor_msgs::ImagePtr out = 
    sensor_msgs::CvBridge::cvToImgMsg(g->getDbgImg(), "rgb8");
  dbgPub.publish(out);

  sensor_msgs::ImagePtr out2 = 
    sensor_msgs::CvBridge::cvToImgMsg(g->person.getScratchImg(), "mono8");
  dptPub.publish(out2);

  if (g->person.isDetected()) posePub.publish(g->person.physPt);

#ifdef WRITEDATA 
  if (g->person.isDetected()) {

    outputFile << debugMsg << "," << g->occupancyString() << "\n";
    //    ROS_INFO(g->occupancyString());

  }
#endif
}

void gracefulClose(int s) {
#ifdef WRITEDATA
  outputFile.close();
#endif
  cout << "gracefulClose caught signal " << s << "\n"; 
  exit(1);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "gests");

  dynamic_reconfigure::Server<gests::gestsConfig> srv;
  dynamic_reconfigure::Server<gests::gestsConfig>::CallbackType f;

  f = boost::bind(&gParamCB, _1, _2);
  srv.setCallback(f);

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  nh.param("gests/personConst", cConst, -1.5);
  //  nh.param("gests/personOffset", cOffset, 125.0);
  nh.param("gests/tolerance", cTolerance, 4);
  nh.param("gests/minSize", cMinSize, 50);
  nh.param("gests/vWidth", cWidth, 640);
  nh.param("gests/vHeight", cHeight, 480);
  nh.param("gests/cameraY", cCameraY, 1.15);
  nh.param<string>("gests/trainingFile", cTrainingFile, "test.csv");

  g = new Gesture(cTrainingFile);


  dbgPub = it.advertise("gests/dbgimg", 1);
  dptPub = it.advertise("gests/dptimg", 1);

  posePub = nh.advertise<geometry_msgs::PoseStamped>("gests/pose",1000);

  ros::Subscriber sub = nh.subscribe("image", 1, processPoints);

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = (void (*)(int))gracefulClose;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

#ifdef WRITEDATA
  outputFile.open("gests-output.txt");

  // Write the file header
  outputFile << "gest,occ1,horiz1,vert1,occ2,horiz2,vert2,occ3,horiz3,vert3,occ4,horiz4,vert4,occ5,horiz5,vert5,occ6,horiz6,vert6,occ7,horiz7,vert7\n";

  ros::Subscriber dsub = nh.subscribe("gests/debug", 1, getDebugMsg);

#endif


  ros::spin();

  return 0;
}
