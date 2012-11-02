#include <ros/ros.h>
#include "sogp.h"

//Message headers
#include "sogp_node/Vector.h"
#include "sogp_node/Matrix.h"
#include "sogp_node/AddVector.h"
#include "sogp_node/AddMatrix.h"

//Service headers
#include "sogp_node/Reset.h"
#include "sogp_node/Predict.h"
#include "sogp_node/SetParameters.h"
#include "sogp_node/PredictVector.h"
#include "sogp_node/PredictMatrix.h"

#include "boost/thread/mutex.hpp"



class SOGPRosNode
{
  //Error message strings
  static const char* m_cszInvalidInputDimension;
  static const char* m_cszInvalidOutputDimension;
  static const char* m_cszInvalidCapacity;
  static const char* m_cszSOGPCreateFailed;
  static const char* m_cszParametersNotSet;
  static const char* m_cszPredictorDimensionDoesNotMatch;

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
  ros::ServiceServer m_srvSetParams;
  ros::ServiceServer m_srvReset;
  ros::ServiceServer m_srvPredictVector;
  ros::ServiceServer m_srvPredictMatrix;

  ///
  ///SOGP objects and parameters
  ///
  SOGP *m_pSOGP;
  RBFKernel* m_pKernel;
  SOGPParams* m_pParams;
  uint m_uiInputDimension;
  uint m_uiOutputDimension;
  uint m_uiCapacity;
  double m_dblW;
  double m_dblN;

public:
  SOGPRosNode();
  void doOneCycle();
  void InitializeConnections();
  bool SetParameters(uint uiInputDimension, uint uiOutputDimension, uint uiCapacity, double dblW, double dblN, const char* &pcszErrorMessage);
  bool InitializeObjects(const char* &pcszErrorMessage);
///
///Service callbacks
///
bool Reset(sogp_node::Reset::Request &req, sogp_node::Reset::Response &resp);
bool SetParameters(sogp_node::SetParameters::Request &req, sogp_node::SetParameters::Response &resp);
bool PredictVector(sogp_node::PredictVector::Request &req, sogp_node::PredictVector::Response &resp);
bool PredictMatrix(sogp_node::PredictMatrix::Request &req, sogp_node::PredictMatrix::Response &resp);

///
///Message callbacks
///
void AddVector(const sogp_node::AddVectorConstPtr& msg);
void AddMatrix(const sogp_node::AddMatrixConstPtr& msg);
};

const char* SOGPRosNode::m_cszInvalidInputDimension = "Parameter initialization failed: dimension of the input vector must be more than 0";
const char* SOGPRosNode::m_cszInvalidOutputDimension = "Parameter initialization failed: dimension of the output vector must be more than 0";
const char* SOGPRosNode::m_cszInvalidCapacity = "Parameter initialization failed: capacity must be more than 0";
const char* SOGPRosNode::m_cszSOGPCreateFailed = "Internal error: failed to create SOGP instance";
const char* SOGPRosNode::m_cszParametersNotSet = "Parameters are not initialized. Initialize parameters first!";
const char* SOGPRosNode::m_cszPredictorDimensionDoesNotMatch = "Dimension of the vector of predictor variables does not match dimension of input data \
specified during the initialization";

SOGPRosNode::SOGPRosNode()
{
  m_pSOGP = NULL;
  m_pKernel = NULL;
  m_pParams = NULL;

  m_uiInputDimension = 0;
  m_uiOutputDimension = 0;
  m_uiCapacity = 0;
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

  m_uiInputDimension = uiInputDimension;
  m_uiOutputDimension = uiOutputDimension;
  m_uiCapacity = uiCapacity;
  m_dblW = dblW;
  m_dblN = dblN;

  printf("Set parameters m_uiInputDimension=%u, m_uiOutputDimension=%u, m_uiCapacity=%u, m_dblW=%f, m_dblN=%f\n",
    m_uiInputDimension, m_uiOutputDimension, m_uiCapacity, m_dblW, m_dblN);

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

  m_pSOGP = new SOGP(m_dblW, m_dblN, m_uiCapacity);
  if (m_pSOGP == NULL)
  {
    pcszErrorMessage = m_cszSOGPCreateFailed;
    return false;
  }

  printf("SOGP object created\n");
  return true;
}

bool SOGPRosNode::Reset(sogp_node::Reset::Request &req, sogp_node::Reset::Response &resp)
{
  const char* pcszErrorMessage;

  if ((m_uiCapacity ==0)||(m_uiInputDimension == 0)||(m_uiOutputDimension==0))
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

//Callback for the SetParameters service 
//Initializes parameters of the SOGP. Must be called prior to sending any training data to SOGP or calling any other service
bool SOGPRosNode::SetParameters(sogp_node::SetParameters::Request &req, sogp_node::SetParameters::Response &resp)
{
  const char* pcszErrorMessage;
  if (!SetParameters(req.input_dimension, 
      req.output_dimension, 
      req.capacity, 
      req.w, 
      req.n, 
      pcszErrorMessage))
  {
    resp.error_msg = pcszErrorMessage;
    printf("ERROR: %s\n", pcszErrorMessage);
    return true;
  }

  if (!InitializeObjects(pcszErrorMessage))
  {
    resp.error_msg = pcszErrorMessage;
    printf("ERROR: %s\n", pcszErrorMessage);
    return true;
  }

  printf("SOGP parameters have been set\n");

  return true;
}

bool SOGPRosNode::PredictVector(sogp_node::PredictVector::Request &req, sogp_node::PredictVector::Response &resp)
{
  ColumnVector predictor(m_uiInputDimension);
  ColumnVector predicted;

  printf("Predicting for vector of a length %u\n", req.predictor.get_data_size());

  if (req.predictor.get_data_size() != m_uiInputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    resp.error_msg = m_cszPredictorDimensionDoesNotMatch;
    printf("ERROR: %s\n", m_cszPredictorDimensionDoesNotMatch);
    return true;
  }

  //Copy message data to the predictor ColumnVectors
  for (uint i=0;i<m_uiInputDimension; i++)
  {
    predictor(i+1) = req.predictor.data[i];
  }
  //Call SOGP for prediction
  predicted = m_pSOGP->predict(predictor);
  //Copy data into the output vector
  resp.prediction.data.resize(predicted.Nrows());
  for(int j = 1; j <= predicted.Nrows(); j++)
  {
    resp.prediction.data[j-1] = predicted(j);
  }

  printf("Predicting done; output dimension is %u\n", resp.prediction.data.size());

  return true;
}

bool SOGPRosNode::PredictMatrix(sogp_node::PredictMatrix::Request &req, sogp_node::PredictMatrix::Response &resp)
{
  ColumnVector predictor(m_uiInputDimension);
  ColumnVector predicted;
  uint uiPredictorVectors;

  printf("Predicting for a matrix with %u rows\n", req.predictor.matrix_rows.size());

  uiPredictorVectors = req.predictor.matrix_rows.size();
  resp.prediction.matrix_rows.resize(uiPredictorVectors);
  for (uint i=0;i< uiPredictorVectors; i++)
  {
    sogp_node::Vector predictorVector = req.predictor.matrix_rows[i];

    if (predictorVector.data.size() != m_uiInputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      resp.error_msg = m_cszPredictorDimensionDoesNotMatch;
      printf("ERROR: %s\n", m_cszPredictorDimensionDoesNotMatch);
      return true;
    }

    //Copy message data to the predictor and target ColumnVectors
    for (uint j=0;j<m_uiInputDimension; j++)
    {
      predictor(j+1) = predictorVector.data[j];
    }

    //Call SOGP for prediction
    predicted = m_pSOGP->predict(predictor);
    //Copy data into the output vector
    sogp_node::Vector predictedVector;
    predictedVector.data.resize(predicted.Nrows());
    for(uint j = 1; j <= predicted.Nrows(); j++)
    {
      predictedVector.data[j-1] = predicted(j);
    }
    //Add the vector to the result matrix
    resp.prediction.matrix_rows[i] = predictedVector;
  }

  printf("Predicting done; output matrix has %u rows\n", resp.prediction.matrix_rows.size());

  return true;
}

void SOGPRosNode::AddVector(const sogp_node::AddVectorConstPtr& msg)
{
  ColumnVector predictor(m_uiInputDimension),target(m_uiOutputDimension);
  printf("Adding predictor vector of a length %u and target vector of a length %u to the model\n", 
    msg->predictor.get_data_size(), msg->target.get_data_size());

  if (msg->predictor.get_data_size() != m_uiInputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    printf("WARNING: length of predictor vector %u is different from input data dimension %u. Skipping...\n", 
      msg->predictor.get_data_size(), m_uiInputDimension);
    return;
  }
  if (msg->target.get_data_size() != m_uiOutputDimension)
  {
    //Skip the vector because its dimension does not match dimensions of the input data 
    //specified by SetParameters service call
    printf("WARNING: length of target vector %u is different from output data dimension %u. Skipping...\n", 
      msg->target.get_data_size(), m_uiOutputDimension);
    return;
  }

  //Copy message data to the predictor and target ColumnVectors
  for (uint i=0;i<m_uiInputDimension; i++)
  {
    predictor(i+1) = msg->predictor.data[i];
  }
  for (uint i=0;i<m_uiOutputDimension; i++)
  {
    target(i+1) = msg->target.data[i];
  }
  //Add them to the SOGP instance
  m_pSOGP->add(predictor, target);

  printf("Predictor vector added\n");
}


void SOGPRosNode::AddMatrix(const sogp_node::AddMatrixConstPtr& msg)
{
  ColumnVector predictor(m_uiInputDimension),target(m_uiOutputDimension);
  printf("Adding predictor matrix with %u rows and target matrix with %u rows to the model\n", 
    msg->predictor.matrix_rows.size(), msg->target.matrix_rows.size());

  uint uiPredictorVectors, uiTargetVectors, uiMinimumSize;
  //get number of rows in both predictor and target matrices
  uiPredictorVectors = msg->predictor.matrix_rows.size();
  uiTargetVectors = msg->target.matrix_rows.size();
  
  //Iterate through the smaller number of rows
  if (uiPredictorVectors < uiTargetVectors)
    uiMinimumSize = uiPredictorVectors;
  else
    uiMinimumSize = uiTargetVectors;
  for (uint i=0;i< uiMinimumSize; i++)
  {
    sogp_node::Vector predictorVector = msg->predictor.matrix_rows[i];
    sogp_node::Vector targetVector = msg->target.matrix_rows[i];

    if (predictorVector.data.size() != m_uiInputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      printf("WARNING: length of %u-th vector in predictor matrix is %u, which is different from input data dimension %u. Skipping...\n", 
        i, predictorVector.data.size(), m_uiInputDimension);
      continue;
    }
    if (targetVector.data.size() != m_uiOutputDimension)
    {
      //Skip the vector because its dimension does not match dimensions of the input data 
      //specified by SetParameters service call
      printf("WARNING: length of %u-th vector in target matrix is %u, which is different from output data dimension %u. Skipping...\n", 
        i, targetVector.data.size(), m_uiOutputDimension);
      continue;
    }

    //Copy message data to the predictor and target ColumnVectors
    for (uint j=0;j<m_uiInputDimension; j++)
    {
      predictor(j+1) = predictorVector.data[j];
    }
    for (uint j=0;j<m_uiOutputDimension; j++)
    {
      target(j+1) = targetVector.data[j];
    }
    //Add them to the SOGP instance
    m_pSOGP->add(predictor, target);
  }

  printf("Predictor matrix added\n");
}

void SOGPRosNode::doOneCycle()
{
}

void SOGPRosNode::InitializeConnections()
{
  //Advertise services
  m_srvReset = m_node.advertiseService("Reset", &SOGPRosNode::Reset, this);
  m_srvSetParams = m_node.advertiseService("SetParameters", &SOGPRosNode::SetParameters, this);
  m_srvPredictVector = m_node.advertiseService("PredictVector", &SOGPRosNode::PredictVector, this);
  m_srvPredictMatrix = m_node.advertiseService("PredictMatrix", &SOGPRosNode::PredictMatrix, this);

  //Subscribe to training messages
  m_subscrAddMatrix = m_node.subscribe("AddMatrix", 1000, &SOGPRosNode::AddMatrix, this);
  m_subscrAddVector = m_node.subscribe("AddVector", 1000, &SOGPRosNode::AddVector, this);
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"sogp_node");

  ros::Rate loop_rate(10);

  SOGPRosNode sogpNode;
  sogpNode.InitializeConnections();

  while(ros::ok())
  {
    sogpNode.doOneCycle();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return(0);
}
