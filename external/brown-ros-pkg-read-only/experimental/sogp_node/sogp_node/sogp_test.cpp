#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <vector>

#include <ros/ros.h>

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

#define DRAND_MAX static_cast<double>(RAND_MAX)
#define RND(max)  static_cast<double>(rand())/(DRAND_MAX/max)

uint uiInputDimension;
uint uiOutputDimension;
uint uiCapacity;
double dblW;
double dblN;

ros::ServiceClient resetClient;
ros::ServiceClient setParamsClient;
ros::ServiceClient predictVectorClient;
ros::ServiceClient predictMatrixClient;
ros::Publisher vectorPublisher;
ros::Publisher matrixPublisher;

void InitializeConnections(ros::NodeHandle nh_)
{
  resetClient = nh_.serviceClient<sogp_node::Reset>("Reset");
  setParamsClient = nh_.serviceClient<sogp_node::SetParameters>("SetParameters");
  predictVectorClient = nh_.serviceClient<sogp_node::PredictVector>("PredictVector");
  predictMatrixClient = nh_.serviceClient<sogp_node::PredictMatrix>("PredictMatrix");

  vectorPublisher = nh_.advertise<sogp_node::AddVector>("AddVector",1000);
  matrixPublisher = nh_.advertise<sogp_node::AddMatrix>("AddMatrix",1000);
}

void ReadParameters(ros::NodeHandle nh_)
{
  int iParam = 0;
  nh_.getParam("InputDimension", iParam, 1);
  uiInputDimension = static_cast<uint>(iParam);
  nh_.getParam("OutputDimension", iParam, 1);
  uiOutputDimension = static_cast<uint>(iParam);
  nh_.getParam("Capacity", iParam, 20);
  uiCapacity = static_cast<uint>(iParam);
  nh_.getParam("W", dblW, 1);
  nh_.getParam("N", dblN, 1);

  printf("InputDimension: %u\n", uiInputDimension);
  printf("OutputDimension: %u\n", uiOutputDimension);
  printf("Capacity: %u\n", uiCapacity);
  printf("W: %f\n", dblW);
  printf("N: %f\n", dblN);
}

void GenerateDatapoint(sogp_node::Vector &predictor, sogp_node::Vector &target, double dblSeed, bool bAddNoisePredict, bool bAddNoiseTarget)
{
  predictor.data.clear();
  target.data.clear();
  double dblRandomNoise = 0;
  predictor.data.resize(uiInputDimension);
  target.data.resize(uiOutputDimension);
  for (uint i=0;i<uiInputDimension; i++)
  {
    if (bAddNoisePredict)
      dblRandomNoise = RND(0.05);
    predictor.data[i] = dblSeed * (i+1) + dblRandomNoise;
    //printf("%f\n", predictor.data[i]);
  }
  for (uint i=0;i<uiOutputDimension; i++)
  {
    if (bAddNoiseTarget)
      dblRandomNoise = RND(0.05);
    target.data[i] = sin(dblSeed * (i+1)) + dblRandomNoise;
  }
}

void GenerateMatrix(sogp_node::Matrix &predictor, sogp_node::Matrix &target, uint uiNumPoints, bool bAddNoisePredict, bool bAddNoiseTarget, double dblSeed, double dblSeedIncrement)
{
  double dblSeedCurrent;
  uint i;

  printf("Preparing matrix with %u rows\n", uiNumPoints);
  
  predictor.matrix_rows.clear();
  target.matrix_rows.clear();
  predictor.matrix_rows.resize(uiNumPoints);
  target.matrix_rows.resize(uiNumPoints);
  for (dblSeedCurrent=dblSeed, i=0; i<uiNumPoints; i++, dblSeedCurrent+=dblSeedIncrement)
  {
    sogp_node::Vector vectPredictor, vectTarget;
    GenerateDatapoint(vectPredictor, vectTarget, dblSeedCurrent, bAddNoisePredict, bAddNoiseTarget);
    predictor.matrix_rows[i] = vectPredictor;
    target.matrix_rows[i] = vectTarget;
  }
}

void InitializeSOGP()
{
  sogp_node::SetParameters params;
  params.request.input_dimension = uiInputDimension;
  params.request.output_dimension = uiOutputDimension;
  params.request.capacity = uiCapacity;
  params.request.w = dblW;
  params.request.n = dblN;

  if (!setParamsClient.call(params))
  {
    printf("Call to to SetParameters service failed\n");
  }
}

double GetMSRE(sogp_node::Vector &vectorPredicted, sogp_node::Vector &vectorValid)
{
  double dblRet = 0;
  if (vectorPredicted.data.size() != vectorValid.data.size())
  {
    printf("Internal error while calculatig MSRE:\n");
    printf("Number of elements in vectorPredicted %ul is different from the number of elements in vectorValid %ul",
      vectorPredicted.data.size(), vectorValid.data.size());
    return -1;
  }
  for (uint j=0; j<vectorPredicted.data.size(); j++)
  {
    //printf("pred - %f, valid - %f", vectorPredicted.data[j], vectorValid.data[j]);
    dblRet += (vectorPredicted.data[j] - vectorValid.data[j])*(vectorPredicted.data[j] - vectorValid.data[j]);
  }
  return sqrt(dblRet);
}

void PrintMatrices(sogp_node::Matrix* pMatrixPredict, sogp_node::Matrix* pMatrixTarget)
{
  for (uint i=0;i<pMatrixPredict->matrix_rows.size(); i++)
  {
    sogp_node::Vector vectorPredictor = pMatrixPredict->matrix_rows[i];
    sogp_node::Vector vectorTarget = pMatrixTarget->matrix_rows[i];
    for (uint j=0;j<vectorPredictor.data.size(); j++)
      printf("%4.4f\t", vectorPredictor.data[j]);

    printf("\t");
    for (uint j=0;j<vectorTarget.data.size(); j++)
      printf("%4.4f\t", vectorTarget.data[j]);

    printf("\n");
  }

}

int main(int argc, char** argv)
{
  srand(time(NULL));

  ros::init(argc,argv,"sogp_test_node");
  ros::NodeHandle nh_;

  InitializeConnections(nh_);
  ReadParameters(nh_);
  
  //Initialize SOGP parameters
  InitializeSOGP();
  sleep(1);
  double dblMSRESum = 0;
  uint i;

  //Train SOGP with set of vectors
  printf("\n\nTrain SOGP with set of vectors. No random noise\n\n");
  sogp_node::AddVector trainVector;
  double dblSeedCurrent = 0.0;
  for (dblSeedCurrent=0.0, i=0; i<50; i++, dblSeedCurrent+=0.05)
  {
    GenerateDatapoint(trainVector.predictor, trainVector.target, dblSeedCurrent, false, false);
    printf("Sending a new training vector to the model\n");
    vectorPublisher.publish(trainVector);
  }

  sleep(3);
  sogp_node::PredictVector predictVector;
  sogp_node::Vector vectorVerify;

  printf("true val");
  for (i=0;i<uiOutputDimension-1;i++)
    printf("\t");
  printf("predicted");
  for (i=0;i<uiOutputDimension-1;i++)
    printf("\t");
  printf("SRE\n");

  for (dblSeedCurrent=0.02, i=0; i<50; i++, dblSeedCurrent+=0.05)
  {
    GenerateDatapoint(predictVector.request.predictor, vectorVerify, dblSeedCurrent, false, false);
    if (!predictVectorClient.call(predictVector))
    {
      printf("Call to PredictVector failed\n");
      return 1;
    }
    if (predictVector.response.error_msg != "")
    {
      printf("Call to PredictVector failed.\nError message %s\n", predictVector.response.error_msg.c_str());
      return 1;
    }

    double dblMSRE = GetMSRE(predictVector.response.prediction, vectorVerify);
    for (uint j=0;j<uiOutputDimension;j++)
      printf("%4.4f ", predictVector.response.prediction.data[j]);
    printf("\t");
    for (uint j=0;j<uiOutputDimension;j++)
      printf("%4.4f ", vectorVerify.data[j]);
    printf("\t%f\n", dblMSRE);
    dblMSRESum += dblMSRE;
  }
  printf("Sum MSRE=%f\n", dblMSRESum/50);

  printf("\nResetting SOGP\n");
  //Reset SOGP node
  sogp_node::Reset resetMsg;
  if (!resetClient.call(resetMsg))
  {
    printf("Call to Reset failed\n");
    return 1;
  }
  if (resetMsg.response.error_msg != "")
  {
    printf("Call to Reset failed.\nError message %s\n", resetMsg.response.error_msg.c_str());
    return 1;
  }

  printf("\n\nTrain SOGP with matrix. Some random noise\n\n");

  //Train SOGP with matrix data
  sogp_node::AddMatrix matrixTrainSet;
  GenerateMatrix(matrixTrainSet.predictor, matrixTrainSet.target, 60, true, true, 0, 0.05);
  //PrintMatrices(&matrixTrainSet.predictor, &matrixTrainSet.target);
  matrixPublisher.publish(matrixTrainSet);

  //Generate test set
  sogp_node::Matrix predictorVerify, targetVerify;
  sogp_node::PredictMatrix predictObj;
  GenerateMatrix(predictObj.request.predictor , targetVerify, 60, true, false, 0.03, 0.05);
  //PrintMatrices(&predictObj.request.predictor, &targetVerify);

  if (!predictMatrixClient.call(predictObj))
  {
    printf("Call to PredictMatrix failed\n");
    return 1;
  }
  if (predictObj.response.error_msg != "")
  {
    printf("Call to Predict failed.\nError message %s\n", predictObj.response.error_msg.c_str());
    return 1;
  }

  sogp_node::Matrix targetPredicted = predictObj.response.prediction;
  if (targetPredicted.matrix_rows.size() != predictObj.request.predictor.matrix_rows.size())
  {
    printf("ERROR: number of rows in the predictor matrix %u is different from the number of rows in the prediction %u\n", 
      predictObj.request.predictor.matrix_rows.size(),
      targetPredicted.matrix_rows.size());
    return 1;
  }

  dblMSRESum = 0;
  printf("true val");
  for (i=0;i<uiOutputDimension-1;i++)
    printf("\t");
  printf("predicted");
  for (i=0;i<uiOutputDimension-1;i++)
    printf("\t");
  printf("SRE\n");

  for (i=0; i<targetPredicted.matrix_rows.size(); i++)
  {
    sogp_node::Vector matrixRowPredicted = targetPredicted.matrix_rows[i];
    sogp_node::Vector matrixRowVerify = targetVerify.matrix_rows[i];
    if (matrixRowPredicted.data.size() != uiOutputDimension)
    {
      printf("ERROR: number of elements in the %ul-th row in the prediction matrix is different from the output dimension %ul\n", 
        matrixRowPredicted.data.size(),
        uiOutputDimension);
      return 1;
    }


    double dblMSRE = GetMSRE(matrixRowPredicted, matrixRowVerify);
    for (uint j=0;j<uiOutputDimension;j++)
      printf("%4.4f ", matrixRowPredicted.data[j]);
    printf("\t");
    for (uint j=0;j<uiOutputDimension;j++)
      printf("%4.4f ", matrixRowVerify.data[j]);
    printf("\t%f\n", dblMSRE);
    dblMSRESum += dblMSRE;

    dblMSRE = GetMSRE(matrixRowPredicted, matrixRowVerify);
    dblMSRESum += dblMSRE;
  }
  printf("MSRE %f\n", dblMSRESum/targetPredicted.matrix_rows.size());

  return(0);
}
