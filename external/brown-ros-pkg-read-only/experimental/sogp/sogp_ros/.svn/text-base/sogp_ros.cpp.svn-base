#include <ros/ros.h>
#include "sogp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//Message headers
#include "sogp/Vector.h"
#include "sogp/Matrix.h"
#include "sogp/AddVector.h"
#include "sogp/AddMatrix.h"

//Service headers
#include "sogp/Reset.h"
#include "sogp/PredictVector.h"
#include "sogp/PredictMatrix.h"

#include "boost/thread/mutex.hpp"


class SOGPRosNode
{
  //Error message strings
  static const char* m_cszInvalidInputDimension;
  static const char* m_cszInvalidOutputDimension;
  static const char* m_cszInvalidCapacity;
  static const char* m_cszSOGPCreateFailed;
  static const char* m_cszParametersNotSet;
  static const char* m_cszInputDimensionDoesNotMatch;

private:
  ///
  /// ROS service information
  ///
  //Node handle
  ros::NodeHandle m_node;
  //Duration of the ROS cycle to process requests
  ros::Duration m_cycletime;
  //Used to synchronize between training and prediction. In fact, if node is processing a topic message,
  //service calls should be postponed and vice versa
  boost::mutex lock;

  ///
  ///Subscribers for training messages
  ///
  ros::Subscriber m_subscrAddMatrix;
  ros::Subscriber m_subscrAddVector;

  ///
  ///Server handlers
  ///

  ros::ServiceServer m_srvReset;
  ros::ServiceServer m_srvPredictVector;
  ros::ServiceServer m_srvPredictMatrix;

  ///
  ///SOGP objects and parameters
  ///
  SOGP *m_pSOGP;
  RBFKernel* m_pKernel;
  SOGPParams* m_pParams;
  int m_iInputDimension;
  int m_iOutputDimension;
  int m_iCapacity;
  double m_dblW;
  double m_dblN;
  FILE *m_loadFile;
  FILE *m_saveFile;
  

public:
  SOGPRosNode();
  void doOneCycle();
  void InitializeConnections();
  bool SetParameters(uint uiInputDimension, uint uiOutputDimension, uint uiCapacity, double dblW, double dblN, const char* &pcszErrorMessage);
  bool InitializeObjects(const char* &pcszErrorMessage);
  void saveFile();
///
///Service callbacks
///
bool Reset(sogp::Reset::Request &req, sogp::Reset::Response &resp);
bool PredictVector(sogp::PredictVector::Request &req, sogp::PredictVector::Response &resp);
bool PredictMatrix(sogp::PredictMatrix::Request &req, sogp::PredictMatrix::Response &resp);

///
///Message callbacks
///
void AddVector(const sogp::AddVectorConstPtr& msg);
void AddMatrix(const sogp::AddMatrixConstPtr& msg);
};

const char* SOGPRosNode::m_cszInvalidInputDimension = "Parameter initialization failed: dimension of the input vector must be more than 0";
const char* SOGPRosNode::m_cszInvalidOutputDimension = "Parameter initialization failed: dimension of the output vector must be more than 0";
const char* SOGPRosNode::m_cszInvalidCapacity = "Parameter initialization failed: capacity must be more than 0";
const char* SOGPRosNode::m_cszSOGPCreateFailed = "Internal error: failed to create SOGP instance";
const char* SOGPRosNode::m_cszParametersNotSet = "Parameters are not initialized. Initialize parameters first!";
const char* SOGPRosNode::m_cszInputDimensionDoesNotMatch = "Dimension of the vector of input variables does not match dimension of input data \
specified during the initialization";

SOGPRosNode::SOGPRosNode()
{
  m_pSOGP = NULL;
  m_pKernel = NULL;
  m_pParams = NULL;

	m_saveFile = NULL;
	m_loadFile = NULL;
  m_iInputDimension = 0;
  m_iOutputDimension = 0;
  m_iCapacity = 0;
  m_dblW = 0.0;
  m_dblN = 0.0;

}

//Initializes SOGP parameters using given values
//pcszErrorMessage contains error message if values of parameters are incorrect or NULL othervise.
//Return value: true if initialization is successful, false othervise
bool SOGPRosNode::SetParameters(uint uiInputDimension, uint uiOutputDimension, uint uiCapacity, double dblW, double dblN, const char* &pcszErrorMessage)
{
  pcszErrorMessage = NULL;
  if (uiInputDimension <= 0)
  {
    pcszErrorMessage = m_cszInvalidInputDimension;
    return false;
  }
  if (uiOutputDimension <= 0)
  {
    pcszErrorMessage = m_cszInvalidOutputDimension;
    return false;
  }
  if (uiCapacity <= 0)
  {
    pcszErrorMessage = m_cszInvalidCapacity;
    return false;
  }

  m_iInputDimension = uiInputDimension;
  m_iOutputDimension = uiOutputDimension;
  m_iCapacity = uiCapacity;
  m_dblW = dblW;
  m_dblN = dblN;

  printf("Set parameters m_iInputDimension=%u, m_iOutputDimension=%u, m_iCapacity=%u, m_dblW=%f, m_dblN=%f\n",
    m_iInputDimension, m_iOutputDimension, m_iCapacity, m_dblW, m_dblN);

  return true;
}

//Create a new SOGP instance and initialize it with parameter values
//pcszErrorMessage contains error message if initialization is unsuccessfull or NULL othervise.
//Return value: true if initialization is successful, false othervise
bool SOGPRosNode::InitializeObjects(const char* &pcszErrorMessage)
{
  if (m_pSOGP != NULL)
  {
    delete m_pSOGP;
    m_pSOGP = NULL;
  }
  m_pSOGP = new SOGP(m_dblW, m_dblN, m_iCapacity);
  if(m_loadFile){
  	m_pSOGP->readFrom(m_loadFile);
  	fclose(m_loadFile);
  	m_loadFile = NULL;
  }
  if (m_pSOGP == NULL)
  {
    pcszErrorMessage = m_cszSOGPCreateFailed;
    return false;
  }

  printf("SOGP object created\n");
  return true;
}

bool SOGPRosNode::Reset(sogp::Reset::Request &req, sogp::Reset::Response &resp)
{
  const char* pcszErrorMessage;

  if ((m_iCapacity ==0)||(m_iInputDimension == 0)||(m_iOutputDimension==0))
  {
    resp.error_msg = m_cszParametersNotSet;
    printf("ERROR: %s\n", m_cszParametersNotSet);
    return true;
  }

  if (!InitializeObjects(pcszErrorMessage))
  {
    resp.error_msg = pcszErrorMessage;
    printf("ERROR: %s\n", pcszErrorMessage);
    return true;
  }

  printf("SOGP object reset\n");

  return true;
}


bool SOGPRosNode::PredictVector(sogp::PredictVector::Request &req, sogp::PredictVector::Response &resp)
{
  ColumnVector input(m_iInputDimension);
  ColumnVector predicted;

  //printf("Predicting for vector of a length %u\n", req.input.get_data_size());

  if (req.input.get_data_size() != (unsigned int)m_iInputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    resp.error_msg = m_cszInputDimensionDoesNotMatch;
    printf("ERROR: %s\n", m_cszInputDimensionDoesNotMatch);
    return true;
  }

  //Copy message data to the input ColumnVectors
  for (int i=0;i<m_iInputDimension; i++)
  {
    input(i+1) = req.input.data[i];
  }
  //Call SOGP for prediction
  predicted = m_pSOGP->predict(input);
  //Copy data into the output vector
  resp.output.data.resize(predicted.Nrows());
  for(int j = 1; j <= predicted.Nrows(); j++)
  {
    resp.output.data[j-1] = predicted(j);
  }

  //printf("Predicting done; output dimension is %u\n", resp.output.data.size());

  return true;
}

bool SOGPRosNode::PredictMatrix(sogp::PredictMatrix::Request &req, sogp::PredictMatrix::Response &resp)
{
  ColumnVector input(m_iInputDimension);
  ColumnVector predicted;
  uint uiInputVectors;

  //printf("Predicting for a matrix with %u rows\n", req.input.matrix_rows.size());

  uiInputVectors = req.input.matrix_rows.size();
  resp.output.matrix_rows.resize(uiInputVectors);
  for (uint i=0;i< uiInputVectors; i++)
  {
    sogp::Vector inputVector = req.input.matrix_rows[i];

    if (inputVector.data.size() != (unsigned int) m_iInputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      resp.error_msg = m_cszInputDimensionDoesNotMatch;
      printf("ERROR: %s\n", m_cszInputDimensionDoesNotMatch);
      return true;
    }

    //Copy message data to the input and output ColumnVectors
    for (int j=0;j<m_iInputDimension; j++)
    {
      input(j+1) = inputVector.data[j];
    }

    //Call SOGP for prediction
    predicted = m_pSOGP->predict(input);
    //Copy data into the output vector
    sogp::Vector predictedVector;
    predictedVector.data.resize(predicted.Nrows());
    for(int j = 1; j <= predicted.Nrows(); j++)
    {
      predictedVector.data[j-1] = predicted(j);
    }
    //Add the vector to the result matrix
    resp.output.matrix_rows[i] = predictedVector;
  }

  printf("Predicting done; output matrix has %u rows\n", resp.output.matrix_rows.size());

  return true;
}

void SOGPRosNode::AddVector(const sogp::AddVectorConstPtr& msg)
{
  ColumnVector input(m_iInputDimension),output(m_iOutputDimension);
  //printf("Adding input vector of a length %u and output vector of a length %u to the model\n", 
    //msg->input.get_data_size(), msg->output.get_data_size());

  if (msg->input.get_data_size() != (unsigned int)m_iInputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    printf("WARNING: length of input vector %u is different from input data dimension %u. Skipping...\n", 
      msg->input.get_data_size(), m_iInputDimension);
    return;
  }
  if (msg->output.get_data_size() != (unsigned int)m_iOutputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    printf("WARNING: length of output vector %u is different from output data dimension %u. Skipping...\n", 
      msg->output.get_data_size(), m_iOutputDimension);
    return;
  }

  //Copy message data to the input and output ColumnVectors
  for (int i=0;i<m_iInputDimension; i++)
  {
    input(i+1) = msg->input.data[i];
  }
  for (int i=0;i<m_iOutputDimension; i++)
  {
    output(i+1) = msg->output.data[i];
  }
  //Add them to the SOGP instance
  m_pSOGP->add(input, output);

  //printf("input vector added\n");
}


void SOGPRosNode::AddMatrix(const sogp::AddMatrixConstPtr& msg)
{
  ColumnVector input(m_iInputDimension),output(m_iOutputDimension);
  printf("Adding input matrix with %u rows and output matrix with %u rows to the model\n", 
    msg->input.matrix_rows.size(), msg->output.matrix_rows.size());

  int uiInputVectors, uioutputVectors, uiMinimumSize;
  //get number of rows in both input and output matrices
  uiInputVectors = msg->input.matrix_rows.size();
  uioutputVectors = msg->output.matrix_rows.size();
  
  //Iterate through the smaller number of rows
  if (uiInputVectors < uioutputVectors)
    uiMinimumSize = uiInputVectors;
  else
    uiMinimumSize = uioutputVectors;
  for (int i=0;i< uiMinimumSize; i++)
  {
    sogp::Vector inputVector = msg->input.matrix_rows[i];
    sogp::Vector outputVector = msg->output.matrix_rows[i];

    if (inputVector.data.size() != (unsigned int) m_iInputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      printf("WARNING: length of %u-th vector in input matrix is %u, which is different from input data dimension %u. Skipping...\n", 
        i, inputVector.data.size(), m_iInputDimension);
      continue;
    }
    if (outputVector.data.size() != (unsigned int)m_iOutputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      printf("WARNING: length of %u-th vector in output matrix is %u, which is different from output data dimension %u. Skipping...\n", 
        i, outputVector.data.size(), m_iOutputDimension);
      continue;
    }

    //Copy message data to the input and output ColumnVectors
    for (int j=0;j<m_iInputDimension; j++)
    {
      input(j+1) = inputVector.data[j];
    }
    for (int j=0;j<m_iOutputDimension; j++)
    {
      output(j+1) = outputVector.data[j];
    }
    //Add them to the SOGP instance
    m_pSOGP->add(input, output);
  }

  printf("input matrix added\n");
}

void SOGPRosNode::doOneCycle()
{
}

void SOGPRosNode::InitializeConnections()
{
  m_node.getParam("/sogp/inputDimension", m_iInputDimension);
  m_node.getParam("/sogp/outputDimension", m_iOutputDimension);
  m_node.getParam("/sogp/capacity", m_iCapacity);
  m_node.getParam("/sogp/width", m_dblW);
  m_node.getParam("/sogp/noise", m_dblN);
  
  std::string fileName;
  if(m_node.getParam("/sogp/loadFile",fileName)){
  	m_loadFile = fopen(fileName.c_str(),"r");
  	printf("Loading SOGP from file %s\n",fileName.c_str());
  } else{ m_loadFile = NULL;
  			printf("Found no load file \n");
  }
  if(m_node.getParam("/sogp/saveFile",fileName)){
  	m_saveFile = fopen(fileName.c_str(),"w");
  } else{ m_saveFile = NULL;
  }
  	
  //Advertise services
  m_srvReset = m_node.advertiseService("Reset", &SOGPRosNode::Reset, this);
  m_srvPredictVector = m_node.advertiseService("PredictVector", &SOGPRosNode::PredictVector, this);
  m_srvPredictMatrix = m_node.advertiseService("PredictMatrix", &SOGPRosNode::PredictMatrix, this);

  //Subscribe to training messages
  m_subscrAddMatrix = m_node.subscribe("AddMatrix", 1000, &SOGPRosNode::AddMatrix, this);
  m_subscrAddVector = m_node.subscribe("AddVector", 1000, &SOGPRosNode::AddVector, this);
  
  const char* pcszErrorMessage;

  if ((m_iInputDimension == 0)||(m_iOutputDimension==0))
  {
    printf("ERROR: %s\n", m_cszParametersNotSet);
  }

  if (!InitializeObjects(pcszErrorMessage))
  {
    printf("ERROR: %s\n", pcszErrorMessage);
  }

}

void SOGPRosNode::saveFile(){
  if(m_saveFile){
  	m_pSOGP->printTo(m_saveFile);
  	fclose(m_saveFile);
  	printf("Saving SOGP with %d datapoints and %d total basis vectors\n",m_pSOGP->total_count,m_pSOGP->current_size);
  }
  m_node.deleteParam("/sogp/saveFile");
  m_node.deleteParam("/sogp/loadFile");
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"sogp");

  ros::Rate loop_rate(10);

  SOGPRosNode sogpNode;
  sogpNode.InitializeConnections();

  while(ros::ok())
  {
    sogpNode.doOneCycle();
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  sogpNode.saveFile();


  return(0);
}
