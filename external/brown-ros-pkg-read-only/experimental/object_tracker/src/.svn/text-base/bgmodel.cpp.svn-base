#include "bgmodel.h"

BGModel::BGModel()
{
  width = height = 0;
  mean = variance = 0;
}

BGModel::~BGModel()
{
  if(mean != 0)
    delete mean;

  if(variance != 0)
    delete variance;
}

void BGModel::save(std::string fileName)
{
  int i;
  std::ofstream fout(fileName.c_str());

  fout << width << " " << height << std::endl;

  fout.precision(4);
  for(i = 0; i < width * height; i++)
    fout << mean[i] << " ";
  fout << std::endl;

  fout.precision(4);
  for(i = 0; i < width * height; i++)
    fout << variance[i]<< " ";

  fout.close();
}

void BGModel::load(std::string fileName)
{
  std::ifstream fin(fileName.c_str());

  if(!fin.is_open())
  {
    ROS_ERROR("Failed to load background model");
    exit(0);
  }

  // getting width and height
  fin >> width >> height;
  
  int i;
  int size = width * height;

  mean = new float[size];
  variance = new float[size];

  for(i = 0; i < size; i++) {
    fin >> mean[i];
  }

  for(i = 0; i < size; i++)
    fin >> variance[i];          

  fin.close();
}

