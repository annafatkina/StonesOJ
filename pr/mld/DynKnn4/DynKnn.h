
#ifndef _DYNAMICAL_KNN_H_
#define _DYNAMICAL_KNN_H_

#include "ANN/Annperf.h"	
#include "ANN/ANn.h"
#include "dynamical.h"
#include <vector>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>




typedef float f32;
typedef std::vector<float> fvec;
typedef std::vector<int> ivec;
typedef int s32;
typedef unsigned int u32;

class DynamicalKNN : public Dynamical
{
private:
	
	u32 dim;
	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTreee;					// search structure
	int metricType;
	int metricP;
	int k;
	std::vector<fvec> points;
	std::vector<fvec> velocities;


public:

	
	DynamicalKNN() : k(1), nPts(0), dataPts(0), nnIdx(0), dists(0), kdTreee(0), metricType(2), metricP(2) { type = DYN_KNN;  };
	~DynamicalKNN();
	
	void Train(std::vector< std::vector<fvec> > trajectories, ivec labels);
	std::vector<fvec> Test(const fvec &sample, const int count);
	fvec Test(const fvec &sample);
	void Draw(IplImage *display);
	char *GetInfoString();

	void SetParams(u32 k, int metricType, u32 metricP);


};



#endif // _DYNAMICAL_KNN_H_
