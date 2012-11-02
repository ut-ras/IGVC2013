#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <vector>

#include <ros/ros.h>

//Message headers
#include "sogp/Vector.h"
#include "sogp/Matrix.h"
#include "sogp/AddVector.h"
#include "sogp/AddMatrix.h"

//Service headers
#include "sogp/Reset.h"
#include "sogp/PredictVector.h"
#include "sogp/PredictMatrix.h"

#define DRAND_MAX static_cast<double>(RAND_MAX)
#define RND(max)  static_cast<double>(rand())/(DRAND_MAX/max)

int m_iInputDimension;
int m_iOutputDimension;
int m_iCapacity;
double m_dblW;
double m_dblN;

ros::ServiceClient resetClient;
ros::ServiceClient predictVectorClient;
ros::ServiceClient predictMatrixClient;
ros::Publisher vectorPublisher;
ros::Publisher matrixPublisher;

void InitializeConnections(ros::NodeHandle nh_)
{
  resetClient = nh_.serviceClient<sogp::Reset>("Reset");
  predictVectorClient = nh_.serviceClient<sogp::PredictVector>("PredictVector");
  predictMatrixClient = nh_.serviceClient<sogp::PredictMatrix>("PredictMatrix");

  vectorPublisher = nh_.advertise<sogp::AddVector>("AddVector",1000);
  matrixPublisher = nh_.advertise<sogp::AddMatrix>("AddMatrix",1000);
}

void ReadParameters(ros::NodeHandle nh_)
{
  nh_.getParam("/sogp/inputDimension", m_iInputDimension);
  nh_.getParam("/sogp/outputDimension", m_iOutputDimension);
  nh_.getParam("/sogp/capacity", m_iCapacity);
  nh_.getParam("/sogp/width", m_dblW);
  nh_.getParam("/sogp/noise", m_dblN);

  printf("InputDimension: %u\n", m_iInputDimension);
  printf("OutputDimension: %u\n", m_iOutputDimension);
  printf("Capacity: %u\n", m_iCapacity);
  printf("W: %f\n", m_dblW);
  printf("N: %f\n", m_dblN);
}

void GenerateDatapoint(sogp::Vector &predictor, sogp::Vector &target, double dblSeed, bool bAddNoisePredict, bool bAddNoiseTarget)
{
  predictor.data.clear();
  target.data.clear();
  double dblRandomNoise = 0;
  predictor.data.resize(m_iInputDimension);
  target.data.resize(m_iOutputDimension);
  for (int i=0;i<m_iInputDimension; i++)
  {
    if (bAddNoisePredict)
      dblRandomNoise = RND(0.05);
    predictor.data[i] = dblSeed * (i+1) + dblRandomNoise;
    //printf("%f\n", predictor.data[i]);
  }
  for (int i=0;i<m_iOutputDimension; i++)
  {
    if (bAddNoiseTarget)
      dblRandomNoise = RND(0.05);
    target.data[i] = sin(dblSeed * (i+1)) + dblRandomNoise;
  }
}

void GenerateMatrix(sogp::Matrix &predictor, sogp::Matrix &target, uint uiNumPoints, bool bAddNoisePredict, bool bAddNoiseTarget, double dblSeed, double dblSeedIncrement)
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
    sogp::Vector vectPredictor, vectTarget;
    GenerateDatapoint(vectPredictor, vectTarget, dblSeedCurrent, bAddNoisePredict, bAddNoiseTarget);
    predictor.matrix_rows[i] = vectPredictor;
    target.matrix_rows[i] = vectTarget;
  }
}


double GetMSRE(sogp::Vector &vectorPredicted, sogp::Vector &vectorValid)
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

void PrintMatrices(sogp::Matrix* pMatrixPredict, sogp::Matrix* pMatrixTarget)
{
  for (uint i=0;i<pMatrixPredict->matrix_rows.size(); i++)
  {
    sogp::Vector vectorPredictor = pMatrixPredict->matrix_rows[i];
    sogp::Vector vectorTarget = pMatrixTarget->matrix_rows[i];
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
  
  sleep(1);
  double dblMSRESum = 0;
  int i;

  //Train SOGP with set of vectors
  printf("\n\nTrain SOGP with set of vectors. No random noise\n\n");
  sogp::AddVector trainVector;
  double dblSeedCurrent = 0.0;
  for (dblSeedCurrent=0.0, i=0; i<50; i++, dblSeedCurrent+=0.05)
  {
    GenerateDatapoint(trainVector.input, trainVector.output, dblSeedCurrent, false, false);
    printf("Sending a new training vector to the model\n");
    vectorPublisher.publish(trainVector);
  }

  sleep(3);
  sogp::PredictVector predictVector;
  sogp::Vector vectorVerify;

  printf("true val");
  for (i=0;i<m_iOutputDimension-1;i++)
    printf("\t");
  printf("predicted");
  for (i=0;i<m_iOutputDimension-1;i++)
    printf("\t");
  printf("SRE\n");

  for (dblSeedCurrent=0.02, i=0; i<50; i++, dblSeedCurrent+=0.05)
  {
    GenerateDatapoint(predictVector.request.input, vectorVerify, dblSeedCurrent, false, false);
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

    double dblMSRE = GetMSRE(predictVector.response.output, vectorVerify);
    for (int j=0;j<m_iOutputDimension;j++)
      printf("%4.4f ", predictVector.response.output.data[j]);
    printf("\t");
    for (int j=0;j<m_iOutputDimension;j++)
      printf("%4.4f ", vectorVerify.data[j]);
    printf("\t%f\n", dblMSRE);
    dblMSRESum += dblMSRE;
  }
  printf("Sum MSRE=%f\n", dblMSRESum/50);

  printf("\nResetting SOGP\n");
  //Reset SOGP node
  sogp::Reset resetMsg;
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
  sogp::AddMatrix matrixTrainSet;
  GenerateMatrix(matrixTrainSet.input, matrixTrainSet.output, 60, true, true, 0, 0.05);
  //PrintMatrices(&matrixTrainSet.predictor, &matrixTrainSet.target);
  matrixPublisher.publish(matrixTrainSet);

  //Generate test set
  sogp::Matrix predictorVerify, targetVerify;
  sogp::PredictMatrix predictObj;
  GenerateMatrix(predictObj.request.input , targetVerify, 60, true, false, 0.03, 0.05);
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

  sogp::Matrix targetPredicted = predictObj.response.output;
  if (targetPredicted.matrix_rows.size() != predictObj.request.input.matrix_rows.size())
  {
    printf("ERROR: number of rows in the predictor matrix %u is different from the number of rows in the prediction %u\n", 
      predictObj.request.input.matrix_rows.size(),
      targetPredicted.matrix_rows.size());
    return 1;
  }

  dblMSRESum = 0;
  printf("true val");
  for (i=0;i<m_iOutputDimension-1;i++)
    printf("\t");
  printf("predicted");
  for (i=0;i<m_iOutputDimension-1;i++)
    printf("\t");
  printf("SRE\n");

  for (i=0; i<(int)targetPredicted.matrix_rows.size(); i++)
  {
    sogp::Vector matrixRowPredicted = targetPredicted.matrix_rows[i];
    sogp::Vector matrixRowVerify = targetVerify.matrix_rows[i];
    if (matrixRowPredicted.data.size() !=  (unsigned int)m_iOutputDimension)
    {
      printf("ERROR: number of elements in the %ul-th row in the prediction matrix is different from the output dimension %ul\n", 
        matrixRowPredicted.data.size(),
         m_iOutputDimension);
      return 1;
    }


    double dblMSRE = GetMSRE(matrixRowPredicted, matrixRowVerify);
    for (int j=0;j< m_iOutputDimension;j++)
      printf("%4.4f ", matrixRowPredicted.data[j]);
    printf("\t");
    for (int j=0;j< m_iOutputDimension;j++)
      printf("%4.4f ", matrixRowVerify.data[j]);
    printf("\t%f\n", dblMSRE);
    dblMSRESum += dblMSRE;

    dblMSRE = GetMSRE(matrixRowPredicted, matrixRowVerify);
    dblMSRESum += dblMSRE;
  }
  printf("MSRE %f\n", dblMSRESum/targetPredicted.matrix_rows.size());

  return(0);
}
