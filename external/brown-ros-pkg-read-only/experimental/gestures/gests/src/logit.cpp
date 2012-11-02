#include "logit.hpp"

void Stump::print() {

  char outbuf[30];

  if (nFeatures > 0) {
    cout << "feature, threshold, sign" << endl;
    for (int i = 0; i < nFeatures; i++) {
      sprintf(outbuf, "    %3d,  %8.5f,  %3d",
	      rows[i].feature, rows[i].threshold, rows[i].sign);
      cout << outbuf << endl;
    }
  } else {
    cout << "nothing here" << endl;
  }
}

void Stump::addFeature(const int newFeature,
		       const double newThreshold,
		       const short newSign) {
  rows.push_back(stumpRow(newFeature, newThreshold, newSign));
  nFeatures++;
}

void Stump::addFeature(const StumpRow inRow) {
  rows.push_back(inRow);
  nFeatures++;
}

void Stump::addFeature(const size_t rowIndex, const StumpRow inRow) {
  if (rowIndex < ((size_t)nFeatures)) {
    rows[rowIndex] = inRow;
  }
}


void GSamples::addSamples(const ublas::matrix<double> xTrain,
			 const vector<string> yTrain) {

  trained = false;
  return;
}

vector<string> GSamples::predict(const ublas::matrix<double>& xSamples) {

  ublas::matrix<double> Prob = predictRaw(xSamples);

  //  int nClass = stumps.size();
  int nTest = xSamples.size1();

  vector<string> out = vector<string>(nTest);

  for (int iTest = 0; iTest < nTest; iTest++) {
    ublas::matrix_row<ublas::matrix<double> > row (Prob, iTest);

    // Just take the class with the maximum probability.  This is
    // sloppy, but we're in hacking mode just now.  If two classes
    // have equal probability, return "NA".  Note that these
    // probabilities are not normalized.
    double maxVal = 0.0;

    for (size_t jClass = 0; jClass < row.size(); jClass++) {
      if (row[jClass] > maxVal) {
	maxVal = row[jClass];
	out[iTest] = classList[jClass];
      } else if (row[jClass] == maxVal) {
	out[iTest] = "NA";
      }
    }

    // int jClass = distance(row.begin(), max_element(row.begin(), row.end()));
    // out[iTest] = classList[jClass];
  }

  return(out);
}


ublas::matrix<double> GSamples::predictRaw(const ublas::matrix<double>& xSamples) {

  int nClass = stumps.size();
  int nTest = xSamples.size1();

  // This will contain the output probabilities.
  ublas::matrix<double> Prob (nTest, nClass);

  // We're going to call predictBinary on each of the stump objects in
  // stumps.  It returns a two-column vector, with column 1 being the
  // probability of that class, and column to being the inverse.
  int iClass = 0;
  for (vector<Stump>::iterator st = stumps.begin(); st < stumps.end(); st++) {
    ublas::matrix<double> prob = predictBinary(xSamples, *st, (*st).getNFeatures());

    // Discard the inverse probability, and assemble it all into a big
    // Prob array, where each column corresponds to the probability of
    // a particular class being the appropriate one for that instance.
    // The probabilities are *not* normalized, so each row does not
    // add to one.
    for (int iTest = 0; iTest < nTest; iTest++) {
      Prob(iTest, iClass) = prob(iTest, 0);
    }
    iClass++;
  }

  return(Prob);
}


// Given a stump object and some data, predict the probability that
// the class represented by that stump is the appropriate class for
// each instance in the input data.
ublas::matrix<double> GSamples::predictBinary(const ublas::matrix<double>& xSamples,
					     const Stump& stump,
					     const int nIter) {

  int nTest = xSamples.size1();

  vector<double> f = vector<double>(nTest);

  // The vector f holds the counts.  The stumps work by providing a
  // list of thresholds and meanings (signs) for each feature.  For
  // each instance, we run through the whole series of thresholds in
  // each stump.  Actually, we're doing it the other way here, and
  // running through all the instances, nIter times.
  for (int iIter = 0; iIter < nIter; iIter++) {
    int iTest = 0;
    for (vector<double>::iterator it = f.begin(); it < f.end(); it++) {
      if (xSamples(iTest++, stump.getFeature(iIter)) <= stump.getThreshold(iIter)) {
	*it += stump.getSign(iIter);
      } else {
	*it -= stump.getSign(iIter);
      }
    }
  }


  // Turn the counts into probability estimates.
  ublas::matrix<double> out (nTest, 2);
  double prob;

  for (int iTest = 0; iTest < nTest; iTest++) {
    prob = 1.0 / (1.0 + exp(-f[iTest]));
    out(iTest, 0) = 1 - prob;
    out(iTest, 1) = prob;
  }

  return(out);
}

// Calls logitBoost, but makes sure that the trained flag is set
// appropriately afterward.
void GSamples::train() {
  if (logitBoost(samples, classes)) {
    if (stumps.size() > 0) {
      trained = stumps[0].real();
    } else {
      trained = false;
    }
  } else {
    cout << "some error" << endl;
  }
}

//  Use this to create the stumps for multiple classes.  The input is
//  the data, plus a string of class names, one for each instance
//  (row) of the data in xTrain.
int GSamples::logitBoost(const ublas::matrix<double>& xTrain,
			 const vector<string>& yTrain) {
  int nClass;

  if (xTrain.size1() < 1) {
    return(0);
  }

  // This is a rule of thumb, but you don't seem to get better results
  // with more iterations.
  int nIter = xTrain.size2();

  // Check if we need to update the class list.  The class list is
  // sorted and unique-ized here.
  if ((classList.size() < 1) || !trained) {
    classList = yTrain;
    sort(classList.begin(), classList.end());

    vector<string>::iterator it;
    it = unique(classList.begin(), classList.end());
    classList.resize(it - classList.begin());
    nClass = classList.size();
  }

  // In general this code is not error-tolerant.  It's designed for
  // reading files that it also writes.
  if (nClass < 2) {
    cout << "What do you think you're doing here?" << endl;
    exit(1);
  }

  // Run through the classes, and call the binary logit on each one.
  for (int jClass = 0; jClass < nClass; jClass++) {

    // cout << "checking class:" << classList[jClass];

    // Create a vector (y) with 0's where the jClass label is, and
    // 1's elsewhere.
    vector <int> y;
    vector<string>::const_iterator jSample;
    for (jSample = yTrain.begin(); jSample < yTrain.end(); jSample++) {
      if (classList[jClass] == *jSample) {
	y.push_back(0);
      } else {
	y.push_back(1);
      }
    }

    // Now train with this binary class.
    stumps.push_back(logitBoostBinary(xTrain, y, nIter));
    stumps.back().setClassName(classList[jClass]);
  }
  return(1);
}

// This is a little hack for sorting an array of index values based on
// the values of another array.
template<class T> struct index_cmp {
  index_cmp(const T arr) : arr(arr) {}
  bool operator()(const size_t a, const size_t b) const {
    return arr[a] < arr[b];
  }
  const T arr;
};


// If we're here, we're training on a binary class.  The yTrain vector
// consists of ones and zeros.  This is the real heart of this work.
Stump GSamples::logitBoostBinary(const ublas::matrix<double>& xTrain,
				const vector<int>& yTrain,
				const int nIter) {

  int nLearn = xTrain.size1();
  int nFeatures = xTrain.size2();

  Stump stump = Stump(nFeatures);

  //  cout << xTrain << endl;
  // for (vector<int>::const_iterator it = yTrain.begin(); it < yTrain.end(); it++)
  //   cout << *it << ",";
  // cout << endl;

  // cout << "nIter: " << nIter << endl;


  // We're going to use this array to calculate thresholds.
  ublas::matrix<double> thresh(nLearn, nFeatures);
  thresh = xTrain;

  for (size_t i = 0; i < thresh.size1(); i++) {
    for (size_t j = 0; j < thresh.size2(); j++) {
      if (isnan(thresh(i,j))) thresh(i,j) = 1.0e35;
    }
  }

  ublas::matrix<size_t> index(nLearn, nFeatures);
  ublas::matrix<bool> mask(nLearn, nFeatures);

  for (int iFeature = 0; iFeature < nFeatures; iFeature++) {

    ublas::matrix_column<ublas::matrix<size_t> > indexCol (index, iFeature);
    // initialize the index column with the ordinal numbers
    size_t i = 0;
    for (ublas::matrix_column< ublas::matrix<size_t> >::iterator it =
	   indexCol.begin();
 	 it < indexCol.end(); it++, i++) *it = i;
    ublas::matrix_column< ublas::matrix<double> > threshCol (thresh, iFeature);

    // cout << "threshCol(unsorted):("; int ij = 0;
    // for (ublas::matrix_column<ublas::matrix<double> >::iterator it = threshCol.begin();
    // 	 it < threshCol.end(); it++)
    //   cout << ij++ << ":" << *it << ",";
    // cout << ")" << endl;

    // First get the index according to the sort.
    sort(indexCol.begin(), indexCol.end(),
	 index_cmp<ublas::matrix_column<ublas::matrix<double> >&>(threshCol));

    // Now do the actual sort.
    sort(threshCol.begin(), threshCol.end());

    // cout << "threshCol:("; int ii = 0;
    // for (ublas::matrix_column<ublas::matrix<double> >::iterator it = threshCol.begin();
    // 	 it < threshCol.end(); it++)
    //   cout << ii++ << ":" << *it << ",";
    // cout << ")" << endl;
    // cout << "indexCol:("; ii = 0;
    // for (ublas::matrix_column<ublas::matrix<size_t> >::iterator it = indexCol.begin();
    // 	 it < indexCol.end(); it++)
    //   cout << ii++ << ":" << *it << ",";
    // cout << ")" << endl;

    // We really only want to look at values that differ from the previous value.
    ublas::matrix_column<ublas::matrix<bool> > maskCol (mask, iFeature);
    for (size_t k = 1; k < maskCol.size(); k++) {
      // TBD: Not sure this is correct.  Seems possible the
      // appropriate mask positions might have the blank at the
      // beginning, not the end.  Future experiment.
      maskCol[k - 1] = ((threshCol[k] - threshCol[k - 1]) != 0);
    }
    maskCol(maskCol.size() - 1) = false; // This is the blank referenced above.

    // ublas::matrix_column< ublas::matrix<double> > tcol (thresh, iFeature);
    // for (ublas::matrix_column< ublas::matrix<double> >::iterator it = tcol.begin();
    // 	 it < tcol.end(); it++) *it = x(1);

  }


  vector<double> p = vector<double>(nLearn); // initialize to 0.5
  for (vector<double>::iterator it = p.begin(); it < p.end(); it++) *it = 0.5;

  vector<int> fints (nLearn);
  int jFeature = -1;

  // Start iteration.  The default value here is just a rule of thumb.
  for (size_t mIter = 0 ; mIter < ((size_t)nIter); mIter++) {
    vector<double> w, z, wz;
    double wsum = 0.0;

    for (int iLearn = 0; iLearn < nLearn; iLearn++) {
      w.push_back( p[iLearn] * (1.0 - p[iLearn]) );
      if (w.back() < 1.0e-24) w.back() = 1.0e-24;

      wsum += w.back();

      z.push_back((yTrain[iLearn] - p[iLearn])/w.back());
    }

    //     w = w/sum(w)
    //     ls1 = sum(w * (z + 1)^2)
    //     ls2 = sum(w * (z - 1)^2)
    //     MinLS = max(ls1, ls2)
    //     wz = 4 * w * z
    double minls, ls1 = 0.0, ls2 = 0.0;
    for (int iLearn = 0; iLearn < nLearn; iLearn++) {
      w[iLearn] = w[iLearn]/wsum;

      wz.push_back(4.0 * w[iLearn] * z[iLearn]);

      ls1 += w[iLearn] * pow((z[iLearn] + 1), 2);
      ls2 += w[iLearn] * pow((z[iLearn] - 1), 2);
    }
    if (ls1 > ls2) {
      minls = ls1;
    } else {
      minls = ls2;
    }

    // cout << "fints:(" ;
    // for (vector<int>::iterator it = fints.begin(); it < fints.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;
    // cout << "p:(" ;
    // for (vector<double>::iterator it = p.begin(); it < p.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;
    // cout << "w:(" ;
    // for (vector<double>::iterator it = w.begin(); it < w.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;
    // cout << "z:(" ;
    // for (vector<double>::iterator it = z.begin(); it < z.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;
    // cout << "yTrain:(" ;
    // for (vector<int>::const_iterator it = yTrain.begin(); it < yTrain.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;



    //     for (iFeat in 1:nFeat) {
    //       if (iFeat == jFeat)
    //         next
    StumpRow newStump = stumpRow();
    for (int iFeature = 0; iFeature < nFeatures; iFeature++) {
      //      cout << minls << endl;

      if (iFeature == jFeature) continue;

      //     Col = Thresh[, iFeat]
      ublas::matrix_column<ublas::matrix<double> > threshCol (thresh, iFeature);
      //     LS = cumsum(wz[Index[, iFeat]])
      ublas::matrix_column<ublas::matrix<size_t> > indexCol (index, iFeature);
      ublas::matrix_column<ublas::matrix<bool> > maskCol (mask, iFeature);


      // cout << "indexCol:("; int ii = 0;
      // for (ublas::matrix_column<ublas::matrix<size_t> >::iterator it = indexCol.begin();
      // 	   it < indexCol.end(); it++)
      // 	cout << ii++ << ":" << *it << ",";
      // cout << ")" << endl;
      // cout << "maskCol:("; ii = 0;
      // for (ublas::matrix_column<ublas::matrix<bool> >::iterator it = maskCol.begin();
      // 	   it < maskCol.end(); it++)
      // 	cout << ii++ << ":" << *it << ",";
      // cout << ")" << endl;


      double cs = 0;
      vector<double> Col, LS;
      //     if (repts[iFeat]) {
      //         mask = Mask[, iFeat]
      //         Col = Col[mask]
      //         LS = LS[mask]
      //     }
      // if (repts[iFeature]) { // not sure this condition is ever false
      // 	cout << "help" << endl;
      // }
      // cout << "wz (index-sorted):(" ;
      // for (int iLearn = 0; iLearn < nLearn; iLearn++)
      // 	cout << iLearn << ":" << wz[indexCol(iLearn)] << ", ";
      // cout << ")" << endl;

      // cout << "cumsum (sorted):(";
      for (int iLearn = 0; iLearn < nLearn; iLearn++) {
	cs += wz[ indexCol(iLearn) ];
	//	cout << iLearn << ":" << cs << ", " ;
	if (maskCol(iLearn)) {
	  LS.push_back(cs);
	  Col.push_back(threshCol(iLearn));
	}
      }
      //      cout << ")" << endl;

      // This trick returns the index of the max or min.
      int iLS1 = distance(LS.begin(), max_element(LS.begin(), LS.end()));
      int iLS2 = distance(LS.begin(), min_element(LS.begin(), LS.end()));

      // cout << "LS:("; int ils = 0;
      // for (vector<double>::iterator it = LS.begin(); it < LS.end(); it++)
      // 	cout << ils++ << ":" << *it << ",";
      // cout << ")" << endl;
      // cout << "Col:("; int icol = 0;
      // for (vector<double>::iterator it = Col.begin(); it < Col.end(); it++)
      // 	cout << icol++ << ":" << *it << ",";
      // cout << ")" << endl;

      double vLS1 = ls1 - LS[iLS1];
      double vLS2 = ls2 + LS[iLS2];

      if (minls > vLS1) {
	newStump.replace(iFeature, Col[iLS1], 1);
	minls = vLS1;
      }
      if (minls > vLS2) {
	newStump.replace(iFeature, Col[iLS2], -1);
	minls = vLS2;
      }

      // cout << "stump:" << newStump.feature << ", " << newStump.threshold;
      // cout << ", " << newStump.sign << endl;

      // cout << "ls1:" << ls1 << endl;
      // cout << "ls2:" << ls2 << endl;

      // cout << "vLS1:" << vLS1 << endl;
      // cout << "vLS2:" << vLS2 << endl;

      // cout << "iLS1:" << iLS1 << endl;
      // cout << "iLS2:" << iLS2 << endl;

      // cout << "minls:" << minls << endl;

    }

    stump.addFeature(mIter, newStump);

    jFeature = newStump.feature;

    // cout << "fints-in:(" ;
    // for (vector<int>::iterator it = fints.begin(); it < fints.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;

    // cout << "m:" << mIter << ",f:" << newStump.feature << ",t:" << newStump.threshold << ",s:" << newStump.sign << endl;

    double conv = 0.0;
    for (int iLearn = 0; iLearn < nLearn; iLearn++) {
      if (xTrain(iLearn, jFeature) <= newStump.threshold) {
	fints[iLearn] += newStump.sign;
      } else {
	fints[iLearn] -= newStump.sign;
      }
    }

    // cout << "fints-out:(" ;
    // for (vector<int>::iterator it = fints.begin(); it < fints.end(); it++)
    //   cout << *it << "," ;
    // cout << ")" << endl;

    for (int iLearn = 0; iLearn < nLearn; iLearn++) {
      p[iLearn] = 1/(1 + exp(-fints[iLearn]));

      double y = 0.0;
      if (fints[iLearn] > 0) {
	y = 1.0;
      } else if (fints[iLearn] == 0) {
	y = 0.5;
      } else {
	y = 0.0;
      }

      // Not currently doing anything with this, but could use it to
      // optimize the choice of nIter, I believe.
      conv += abs(((double)yTrain[iLearn]) - y);
    }

    //cout << "CONV:" << conv << endl;
  }

  stump.print();

  return(stump);
}

int GSamples::readFile(const string fileName) {

  ifstream inputFile;
  string inBuffer, lbuf;
  vector< vector<double> > matBuffer;

  inputFile.open(fileName.c_str());

  if (inputFile.is_open()) {
    // get header
    getline(inputFile, inBuffer);

    // Process the pieces of the header.  They are *unquoted* strings.
    stringstream headerInput;
    headerInput << inBuffer;

    int nFeatures = 0;
    while (getline(headerInput, lbuf, ',')) {
      if (nFeatures > 0) header.push_back(lbuf);
      nFeatures++;
    }
    // We don't need the heading of the first column.
    nFeatures--;

    // for (vector<string>::iterator it = header.begin(); it < header.end(); it++)
    //   cout << *it << "****" << endl;
    // cout << "nf:" << nFeatures;

    // As of here, the header is parsed.  Now get all the samples.
    while (inputFile.good())   {
      getline(inputFile, inBuffer);
      if (inBuffer.size() == 0) break;

      stringstream lineInput;
      lineInput << inBuffer;

      vector<double> lineData;
      for (int i = 0; i <= nFeatures; i++) {
	getline(lineInput, lbuf, ',');
	if (i) {
	  lineData.push_back(atof(lbuf.c_str()));
	} else {
	  classes.push_back(lbuf.c_str());
	}
      }
      matBuffer.push_back(lineData);
    }

    inputFile.close();

    samples.resize(matBuffer.size(), matBuffer[0].size());
    for (unsigned int i = 0; i < matBuffer.size(); i++) {
      for (unsigned int j = 0; j < matBuffer[0].size(); j++) {
	samples(i,j) = matBuffer[i][j];
      }
    }
    return(1);

  } else {

    cout << "Problem opening file." << endl;
    return(0);
  }
}

// Debugging code to print out the samples data.
void GSamples::print() {

  printMatrix(samples, classes);

}

void GSamples::printMatrix(const ublas::matrix<double>& mat, const vector<string>& names) {

  char buf[10];

  for (size_t iRow = 0; iRow < mat.size1(); iRow++) {

    // Don't know why the copying is necessary.  Seems I can't
    // figure out the correct spelling of the matrix_row line.
    ublas::matrix<double> matCopy = mat;
    ublas::matrix_row<ublas::matrix<double> > row (matCopy, iRow);

    unsigned int maxlen = 0;
    for (vector<string>::const_iterator it = names.begin(); it < names.end(); it++)
      if ((*it).size() > maxlen) maxlen = (*it).size();

    sprintf(buf, " %*s:(", maxlen, names[iRow].c_str());
    cout << buf;
    for (ublas::matrix_row<ublas::matrix<double> >::iterator jCol = row.begin();
	 jCol < row.end(); jCol++) {
      sprintf(buf, "%8.5f, ", *jCol);
      cout << buf ;
    }
    cout << ")" << endl;
  }
}

// Testing code.

// int main(int argc, char** argv) {

//   GSamples s = GSamples(argv[1]);

//   //  s.print();

//   s.train();

//   GSamples t = GSamples(argv[2]);
//   ublas::matrix<double> u = t.getSamples();
//   vector<string> v = t.getClasses();

//   vector<string> c = s.predict(u);

//   // for (size_t i = 0; i < c.size(); i++) {
//   //   cout << c[i] << "<--" << v[i] << endl ;
//   // }

//   ublas::matrix<double> w = s.predictRaw(u);

//   vector<string> x = s.getClassList();
//   for (vector<string>::iterator it = x.begin(); it < x.end(); it++) {
//     cout << *it << ",";
//   }
//   cout << endl;

//   char buf[20];
//   vector<string> combine;
//   for (size_t i = 0; i < c.size(); i++) {
//     sprintf(buf, "%s(%s)", c[i].c_str(), v[i].c_str());
//     combine.push_back(string(buf));
//   }

//   s.printMatrix(w, combine);

// }

