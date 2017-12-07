#ifndef _SOURCE_H_
#define _SOURCE_H_

#include "ANN/ANn.h"
#include "dynamical.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include "DynKnn.h"

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


typedef float f32;
typedef std::vector<float> fvec;
typedef std::vector<int> ivec;
typedef int s32;
typedef unsigned int u32;
using namespace std;
using namespace cv;
class DynKnn4
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
	vector<fvec> points;
	vector<fvec> velocities;
public:
	
	fvec Test(Dynamical *dynamical, vector< vector<fvec> > trajectories, ivec labels);
	void Train(vector< vector<fvec> > trajectories, ivec labels);
	fvec Train(Dynamical *dynamical, vector<fvec> samples, vector< ipair > sequences, ivec labels);
	vector< fvec > samples;
	ipair trajectory;
	int size;
	Dynamical *dynamical = new Dynamical;
	void Dynamize(vector<fvec> &samples, vector< ipair > &sequences, ivec &labels);
	vector< vector < fvec > > GetTrajectories(vector<fvec> samples, vector< ipair > sequences, ivec labels);
	/*void Train(DynamicalKNN *dynamical){
		{
			if (!dynamical) return;
			//std::vector<fvec> samples = Canvas::data.GetSamples();
			std::vector<fvec> samples = { { 1, 2 }, { 1, 3 }, { 1, 4 } };
			std::vector<ipair> sequences = { { 1, 2 }, { 1, 3 }, { 1, 4 } };
			//ivec labels = Canvas::data.GetLabels();
			ivec labels = { 1, 1, 1 };
			//if (!samples.size() || !sequences.size()) return;
			int dim = samples[0].size();
			int count = 5;//dynOptions->resampleSpin->value();
			int resampleType = 0;//dynOptions->resampleCombo->currentIndex();
			int centerType = 0;// dynOptions->centerCombo->currentIndex();
			bool zeroEnding = 0;// dynOptions->zeroCheck->isChecked();

			// we split the data into trajectories
			std::vector< std::vector<fvec> > trajectories;
			ivec trajLabels;
			trajectories.resize(sequences.size());
			trajLabels.resize(sequences.size());
			for (int i = 0; i<sequences.size(); i++)
			{
				int length = sequences[i].second - sequences[i].first + 1;
				trajLabels[i] = { 1 }; //Canvas::data.GetLabel(sequences[i].first);
				trajectories[i].resize(length);
				for (int j = 0; j< length; j++)
				{
					trajectories[i][j].resize(dim * 2);
					// copy data
					for (int d = 0; d< dim; d++) trajectories[i][j][d] = samples[sequences[i].first + j][d];
				}
			}

			switch (resampleType)
			{
			case 0: // none
			{
				for (int i = 0; i<sequences.size(); i++)
				{
					int cnt = sequences[i].second - sequences[i].first + 1;
					if (count > cnt) count = cnt;
				}
				for (int i = 0; i<trajectories.size(); i++)
				{
					while (trajectories[i].size() > count) trajectories[i].pop_back();
				}
			}
			break;
			case 1: // uniform
			{
				for (int i = 0; i<trajectories.size(); i++)
				{
					std::vector<fvec> trajectory = trajectories[i];
					trajectories[i] = interpolate(trajectory, count);
				}
			}
			break;
			}


			if (centerType)
			{
				std::map<int, int> counts;
				std::map<int, fvec> centers;
				for (int i = 0; i<sequences.size(); i++)
				{
					int index = centerType ? sequences[i].second : sequences[i].first; // start
					int label = 1;//Canvas::data.GetLabel(index);
					if (!centers.count(label))
					{
						fvec center;
						center.resize(2, 0);
						centers[label] = center;
						counts[label] = 0;
					}
					centers[label] += samples[index];
					counts[label]++;
				}
				for (map<int, int>::iterator p = counts.begin(); p != counts.end(); ++p)
				{
					int label = p->first;
					centers[label] /= p->second;
				}
				for (int i = 0; i<trajectories.size(); i++)
				{
					fvec difference = centers[trajLabels[i]] - trajectories[i][count - 1];
					for (int j = 0; j< count; j++) trajectories[i][j] += difference;
				}
			}

			float dT = 10.f; // time span between each data frame
			dynamical->dT = dT;

			float maxV = -FLT_MAX;
			// we compute the velocity
			for (int i = 0; i<trajectories.size(); i++)
			{
				for (int j = 0; j< count - 1; j++)
				{
					for (int d = 0; d< dim; d++)
					{
						float velocity = (trajectories[i][j + 1][d] - trajectories[i][j][d]) * dT;
						trajectories[i][j][dim + d] = velocity;
						if (velocity > maxV) maxV = velocity;
					}
				}
				if (!zeroEnding)
				{
					for (int d = 0; d< dim; d++)
					{
						trajectories[i][count - 1][dim + d] = trajectories[i][count - 2][dim + d];
					}
				}
			}

			for (int i = 0; i<trajectories.size(); i++)
			{
				for (int j = 0; j< count; j++)
				{
					for (int d = 0; d< dim; d++)
					{
						trajectories[i][j][dim + d] /= maxV;
					}
				}
			}

			dynamical->Traind(trajectories, labels);
		}
	}*/
	void AddSample(fvec sample, vector<fvec>  &samples /*, dsmFlags flag*/);
	vector< ipair > AddSequence(ipair newSequence, vector< ipair > &sequences);
	int GetLabel(int index);
	void Draw(Dynamical *dynamical);
	


};

int mldcall(vector<vector<Point>>);

#endif // _SOURCE_H_
