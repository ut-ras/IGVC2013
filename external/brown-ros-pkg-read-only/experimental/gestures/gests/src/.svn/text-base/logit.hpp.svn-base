// A lightweight version of the logitboost algorithm.  See the
// comments below for the GSamples class for usage instructions.
//
// Tom Sgouros 10/2012
//
#ifndef LOGIT_H
#define LOGIT_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std;
namespace ublas=boost::numeric::ublas;

// This class exists just to lump the three values that make up a
// stump entry.
typedef struct stumpRow {
  int feature;
  double threshold;
  short sign;

  stumpRow(int inFeature, double inThreshold, short inSign) :
    feature(inFeature), threshold(inThreshold), sign(inSign) {}
  stumpRow() :
    feature(0), threshold(0.0), sign(0) {}

  void replace(int inFeature, double inThreshold, short inSign) {
    feature = inFeature;
    threshold = inThreshold;
    sign = inSign;
  }

} StumpRow;

// A stump is essentially a long filter used by the classification
// code in the GSamples::predict* methods.  Each row defines a feature
// to check out, a threshold to which to compare it, and a meaning
// (sign) of the threshold.  A collection of these rows makes the
// whole filter.  The logit method is a way to build such a filter.
class Stump {

  int nFeatures;
  vector<StumpRow> rows;
  string className;

public:
  Stump(const int inFeatures) : nFeatures(inFeatures) {
    rows = vector<StumpRow>(nFeatures);
  };

  bool real() {
    return((nFeatures > 0));
  };

  void print();

  void addFeature(const int newFeature,
		  const double newThreshold,
		  const short newSign);
  void addFeature(const StumpRow newRow);

  void addFeature(const size_t rowIndex, const StumpRow newRow);

  int getLastFeature() const { return rows.back().feature; }
  double getLastThreshold() const { return rows.back().threshold; }
  short getLastSign() const { return rows.back().sign; }

  int getFeature(const int index) const { return rows.at(index).feature; }
  double getThreshold(const int index) const {
    return rows.at(index).threshold;
  }
  short getSign(const int index) const { return rows.at(index).sign; }

  int getNFeatures() const { return nFeatures; }

  string getClassName() const { return className; }
  void setClassName(const string newClassName) { className = newClassName; }
};


// Class to hold sample data, perform a logistic regression on it, and
// offer classifications of similar data on it afterward.
//
// To use this class:
//    - Read some data from a file by initializing this class with a
//      file name (file format documented in the .cpp file at readFile
//    - Run train() on it
//    - Feed it some other data, to get a prediction of which class
//      that data belongs in.  Use predict() or predictRaw()
class GSamples {

private:

  int nFeatures;
  // Contains the names of the features.  The column headings of the
  // samples matrix.
  vector<string> header;
  // Each row is a sample (also called an instance) and each column
  // represents a feature or field.
  ublas::matrix<double> samples;
  // A string for each instance.  The point of this code is to learn
  // the mapping between the x (a.k.a. the instances or samples) and
  // these class names.  After learning the mapping, we will use the
  // learned correlations to make predictions of these classes based
  // on new instances.
  vector<string> classes;

  // This is the list of all the possible classes we're looking for.
  vector<string> classList;
  int nClass;

  // A vector of the Stump objects corresponding to the classes in the
  // above classList.
  vector<Stump> stumps;

  // A flag to indicate whether we've made a training run on this set
  // of samples or not.
  bool trained;

public:
  GSamples() : trained(false) {}

  GSamples(const string fileName) {
    readFile(fileName);
  };

  // Reads data from a CSV file.  A header is assumed to be present,
  // and a class name for each row.  The class name column has a
  // header entry, too, but it is ignored.  e.g.:
  // gest, occ1, horiz1, vert1, occ2, horiz2...
  // forward, 0.34, -0.012, 0.012, 0.3, -0.002, 0.33...
  // stop, 0.01, 0.02, 0.001, 0.02, -0.022, 0.44
  int readFile(const string fileName);
  void print();
  // Utility function used by print(), but also available for debugging.
  void printMatrix(const ublas::matrix<double>& mat,
		   const vector<string>& names);

  ublas::matrix<double> getSamples() { return samples; }
  vector<string> getClasses() { return classes; }
  vector<string> getClassList() { return classList; }

  // Training the model on the data is kept separate from reading in
  // the data solely for optimization purposes.  That is, in practice,
  // I'm not sure how much of a load it's going to be, so the calling
  // program may want to have control over when it happens.
  void train();
  bool isTrained() { return trained; }

  // This is the C++ translation of the logitBoost R function.
  int logitBoost(const ublas::matrix<double>& xTrain,
		 const vector<string>& yTrain);
  Stump logitBoostBinary(const ublas::matrix<double>& xTrain,
			 const vector<int>& yTrain,
			 const int nClass);

  // Add samples to the training mix with this function.
  void addSamples(const ublas::matrix<double> xTrain,
		  const vector<string> yTrain);

  // Returns a table of probabilities, each row corresponding to the
  // sample in the same row of the input array, and each column being
  // the probability that the corresponding class (from classList) is
  // the right one.
  ublas::matrix<double> predictRaw(const ublas::matrix<double>& xSamples);

  // This is a hack that just takes the most probable class in each
  // row and returns a vector of class names corresponding to the
  // input instances (rows of the input array.
  vector<string> predict(const ublas::matrix<double>& xSamples);

  ublas::matrix<double> predictBinary(const ublas::matrix<double>& xSamples,
				      const Stump& stump,
				      const int nIter);

};


#endif
