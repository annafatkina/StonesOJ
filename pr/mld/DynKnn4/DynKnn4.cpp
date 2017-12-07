// DynKnn4.cpp : Defines the entry point for the console application.
//

#include "ANN/ANn.h"
#include <fstream>
#include <string.h>
#include <stdio.h>
#include "DynKnn.h"
#include "mymath.h"
#include "DynKnn4.h"
#include "dynamical.h"
#include <vector>
//#include <conio.h>


#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define drand48() (rand()/(float)RAND_MAX)


using namespace std;
using namespace cv;

int GetLabel(int index, ivec labels){ return index < labels.size() ? labels[index] : 0; }


fvec Test(Dynamical *dynamical, vector< vector<fvec> > trajectories, ivec labels)
{
	if (!dynamical || !trajectories.size()) return fvec();
	int dim = trajectories[0][0].size() / 2;
	//(int dim = dynamical->Dim();
	float dT = dynamical->dT;
	fvec sample; sample.resize(dim, 0);
	fvec vTrue; vTrue.resize(dim, 0);
	fvec xMin(dim, FLT_MAX);
	fvec xMax(dim, -FLT_MAX);
	// test each trajectory for errors
	int errorCnt = 0;
	float errorOne = 0, errorAll = 0;
	for (int i = 0; i < trajectories.size();i++)
	{
		vector<fvec> t = trajectories[i];
		float errorTraj = 0;
		for (int j = 0; j < t.size();j++)
		{
			for (int d = 0; d< dim;d++)
			{
				sample[d] = t[j][d];
				vTrue[d] = t[j][d + dim];
				if (xMin[d] > sample[d]) xMin[d] = sample[d];
				if (xMax[d] < sample[d]) xMax[d] = sample[d];
			}
			fvec v = dynamical->Test(sample);
			float error = 0;
			for (int d = 0; d < dim;d++) error += (v[d] - vTrue[d])*(v[d] - vTrue[d]);
			errorTraj += error;
			errorCnt++;
		}
		errorOne += errorTraj;
		errorAll += errorTraj / t.size();
	}
	errorOne /= errorCnt;
	errorAll /= trajectories.size();
	fvec res;
	res.push_back(errorOne);
	vector<fvec> endpoints;
	float errorTarget = 0;
	// test each trajectory for target
	fvec pos(dim), end(dim);
	for (int i = 0; i < trajectories.size();i++)
	{
		for (int d = 0; d < dim; d++)
		{
			pos[d] = trajectories[i].front()[d];
			end[d] = trajectories[i].back()[d];
		}
		if (!endpoints.size()) endpoints.push_back(end);
		else
		{
			bool bExists = false;
			for (int j = 0; j < endpoints.size();j++)
			{
				if (endpoints[j] == end)
				{
					bExists = true;
					break;
				}
			}
			if (!bExists) endpoints.push_back(end);
		}
		int steps = 500;
		float eps = FLT_MIN;
		for (int j = 0; j < steps; j++)
		{
			fvec v = dynamical->Test(pos);
			float speed = 0;
			for (int d = 0; d < dim; d++) speed += v[d] * v[d];
			speed = sqrtf(speed);
			if (speed*dT < eps) break;
			pos += v*dT;
		}
		float error = 0;
		for (int d = 0; d < dim; d++)
		{
			error += (pos[d] - end[d])*(pos[d] - end[d]);
		}
		error = sqrtf(error);
		errorTarget += error;
	}
	errorTarget /= trajectories.size();
	res.push_back(errorTarget);
	fvec xDiff = xMax - xMin;
	errorTarget = 0;
	int testCount = 30;
	cout << "Wait a sec"<<endl;
	for (int i = 0; i < testCount;i++)
	{
		for (int d = 0; d < dim; d++)
		{
			pos[d] = ((drand48() * 2 - 0.5)*xDiff[d] + xMin[d]);
		}
		int steps = 500;
		float eps = FLT_MIN;
		for (int j = 0; j < steps; j++)
		{
			fvec v = dynamical->Test(pos);
			float speed = 0;
			for (int d = 0; d < dim; d++) speed += v[d] * v[d];
			speed = sqrtf(speed);
			if (speed*dT < eps) break;
			pos += v*dT;
		}
		float minError = FLT_MAX;
		for (int j = 0; j < endpoints.size();j++)
		{
			float error = 0;
			for (int d = 0; d < dim; d++)
			{
				error += (pos[d] - endpoints[j][d])*(pos[d] - endpoints[j][d]);
			}
			error = sqrtf(error);
			if (minError > error) minError = error;
		}
		errorTarget += minError;
	}
	errorTarget /= testCount;
	res.push_back(errorTarget);
	return res;
}



vector< vector < fvec > > GetTrajectories(vector<fvec> samples, vector< ipair > sequences, ivec labels)
{
	int resampleType = 0;// optionsDynamic->resampleCombo->currentIndex();
	int resampleCount = 100;// optionsDynamic->resampleSpin->value();
	int centerType = 0;// optionsDynamic->centerCombo->currentIndex();
	float dT = 10.f;// optionsDynamic->dtSpin->value();
	int zeroEnding = 0;// optionsDynamic->zeroCheck->isChecked();
	bool bColorMap = 0;// optionsDynamic->colorCheck->isChecked();

	// we split the data into trajectories
	vector< vector<fvec> > trajectories;
	if (!sequences.size() || !samples.size()) return trajectories;
	int dim = samples[0].size();
	trajectories.resize(sequences.size());
	for (int i = 0; i < sequences.size(); i++)
	{
		
		int length = sequences[i].second - sequences[i].first + 1;
		trajectories[i].resize(length);
		for (int j = 0; j < length; j++)
		{

			trajectories[i][j].resize(dim * 2);
			// copy data
			for (int d = 0; d < dim; d++) { trajectories[i][j][d] = samples[sequences[i].first + j][d];  }
		}
		
	}

	switch (resampleType)
	{
	case 0: // none
	{
		for (int i = 0; i< sequences.size(); i++)
		{
			int cnt = sequences[i].second - sequences[i].first + 1;
			if (resampleCount > cnt) resampleCount = cnt;
		}
		for (int i = 0; i < trajectories.size(); i++)
		{
			while (trajectories[i].size() > resampleCount) trajectories[i].pop_back();
		}
	}
	break;
	case 1: // uniform
	{
		for (int i = 0; i < trajectories.size(); i++)
		{
			vector<fvec> trajectory = trajectories[i];
			trajectories[i] = interpolate(trajectory, resampleCount);
		}
	}
	break;

	break;
	}


	if (centerType)
	{
		map<int, int> counts;
		map<int, fvec> centers;
		vector<int> trajLabels(sequences.size());
		for (int i = 0; i< sequences.size(); i++)
		{
			int index = centerType == 1 ? sequences[i].second : sequences[i].first; // start
			int label = GetLabel(index, labels);
			trajLabels[i] = label;
			if (!centers.count(label))
			{
				fvec center(dim, 0);
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
		for (int i = 0; i < trajectories.size(); i++)
		{
			if (centerType == 1)
			{
				fvec difference = centers[trajLabels[i]] - trajectories[i].back();
				for (int j = 0; j < resampleCount; j++) trajectories[i][j] += difference;
			}
			else
			{
				fvec difference = centers[trajLabels[i]] - trajectories[i][0];
				for (int j = 0; j < resampleCount; j++) trajectories[i][j] += difference;
			}
		}
	}

	float maxV = -FLT_MAX;
	// we compute the velocity
	for (int i = 0; i < trajectories.size(); i++)
	{
		for (int j = 0; j < resampleCount - 1; j++)
		{
			for (int d = 0; d< dim; d++)
			{
				float velocity = (trajectories[i][j + 1][d] - trajectories[i][j][d]) / dT;
				trajectories[i][j][dim + d] = velocity;
				if (velocity > maxV) maxV = velocity;

			}

		}
		if (!zeroEnding)
		{
			for (int d = 0; d< dim; d++)
			{
				trajectories[i][resampleCount - 1][dim + d] = trajectories[i][resampleCount - 2][dim + d];
			}
		}
	}

	// we normalize the velocities as the variance of the data
	fvec mean, sigma;
	mean.resize(dim, 0);
	int cnt = 0;
	sigma.resize(dim, 0);
	for (int i = 0; i < trajectories.size(); i++)
	{
		for (int j = 0; j < resampleCount; j++)
		{
			mean += trajectories[i][j];
			cnt++;
		}
	}
	mean /= cnt;
	for (int i = 0; i < trajectories.size(); i++)
	{
		for (int j = 0; j < resampleCount; j++)
		{
			fvec diff = (mean - trajectories[i][j]);
			for (int d = 0; d< dim; d++) sigma[d] += diff[d] * diff[d];
		}
	}
	sigma /= cnt;

	for (int i = 0; i < trajectories.size(); i++)
	{
		for (int j = 0; j < resampleCount; j++)
		{
			for (int d = 0; d< dim; d++)
			{
				trajectories[i][j][dim + d] /= maxV;
				//trajectories[i][j][dim + d] /= sqrt(sigma[d]);
			}
		}
	}
	return trajectories;
}


fvec Train(Dynamical *dynamical, vector<fvec> samples, vector< ipair > sequences, ivec labels)
{
	if(!dynamical) return fvec();
	
	if (!samples.size() || !sequences.size()) return fvec();
	int dim = samples[0].size();
	int resampleType = 0;// optionsDynamic->resampleCombo->currentIndex();
	int count = 100;// optionsDynamic->resampleSpin->value();
	int centerType = 0;// optionsDynamic->centerCombo->currentIndex();
	float dT = 10.f;// optionsDynamic->dtSpin->value();
	int zeroEnding = 0;// optionsDynamic->zeroCheck->isChecked();
	
	
	ivec trajLabels(sequences.size());
	for (int i = 0; i < sequences.size(); i++)
	{
		trajLabels[i] = labels[sequences[i].first];
	}

	//float dT = 10.f; // time span between each data frame
	
	dynamical->dT = dT;
	//dT = 10.f;
	vector< vector<fvec> > trajectories = GetTrajectories(samples, sequences, labels);
	interpolate(trajectories[0], count);
	
	
	dynamical->Train(trajectories, labels);
	return Test(dynamical, trajectories, labels);

}



float tofloat(fvec suka,int i){
	 float aa = suka[i];
	 return aa;
}



void Dynamize(vector<fvec> &samples, vector< ipair > &sequences, ivec &labels)
{
	
	ofstream fouttt;
	float firstcord; float secondcord;
	firstcord = 0; secondcord = 0;
	fouttt.open("Points.xyz");
	cout << "Dynamize"<<endl;
	DynamicalKNN *dynamical = new DynamicalKNN();
	((DynamicalKNN *)dynamical)->SetParams(2, 1, 5);//(k, metricType, metricP);
	Train(dynamical,samples,sequences,labels);
	//dynamicals[tab]->Draw(canvas, dynamical);

	//int w = canvas->width(), h = canvas->height();
	for (int xdim = 70; xdim < 150; xdim++){
		for (int ydim = 70; ydim < 150; ydim++){
	ivec labels1 = {1};
	vector<fvec> samples1 = { { float(xdim), float(ydim) }, { float(xdim + 1), float(ydim+1) } };
	vector< ipair > sequences1 = { { 0, 1 } };

	// we draw the current trajectories
	
	vector< vector<fvec> > trajectories = GetTrajectories(samples1, sequences1, labels1);
		//GetTrajectories(samples, sequences, labels); 
	
	vector< vector<fvec> > testTrajectories(4);
	int steps = 3;
	if (trajectories.size())
	{
		testTrajectories.resize(trajectories.size());
		int dim = trajectories[0][0].size() / 2;
		for (int i = 0; i < trajectories.size(); i++)
		{
			fvec start(dim, 0);
			for (int d = 0; d < dim; d++) start[d] = trajectories[i][0][d];
			vector<fvec> result = dynamical->Test(start, steps);
			testTrajectories[i] = result;


		}

	
        std::cout << "tr size = " << testTrajectories.size() << std::endl;

		for (int i = 0; i < testTrajectories.size(); i++)
		{
			vector<fvec> &result = testTrajectories[i];
			fvec oldPt = result[0];
			int count = result.size();
            std::cout << "count = " << count << std::endl;
			for (int j = 1; j < count - 1; j++)
			{
				fvec pt = result[j + 1];

				
				firstcord = tofloat(oldPt, 0); secondcord = tofloat(oldPt, 1);
				fouttt << firstcord << " " <<secondcord << " " << 0 << std::endl;// << " " << oldPt[1] << " " << pt[0] << " " << pt[1];
				firstcord = tofloat(pt, 0); secondcord = tofloat(pt, 1);
				fouttt << firstcord << " " << secondcord << " " << 0 << std::endl;
			//	cout << "[" << oldPt[0] << "," << oldPt[1] << "]->[" << pt[0] << "," << pt[1] << "]" << endl;
				oldPt = pt;
			}
			}
		}
	}
	
	}
	fouttt << -1000 << "\n" << -1000 << "\n"; fouttt << -1000 << "\n" << -1000 << "\n";
	fouttt.close();

	// the first index is "none", so we subtract 1
}



void AddSample(fvec sample, vector<fvec>  &samples /*, dsmFlags flag*/)
{
	if (!sample.size()) return;
	int size = sample.size();

	samples.insert(samples.end(), sample);
	
}
void AddSequence(ipair newSequence, vector< ipair > &sequences)
{
	//if (newSequence.first >= samples.size() || newSequence.second >= samples.size()) return;
	
	sequences.push_back(newSequence);
	// sort sequences by starting value
	sort(sequences.begin(), sequences.end());
	
}


//int main()
int mldcall(vector<vector<Point>> contours)
{
	DynamicalKNN *dyn = new DynamicalKNN;
	//std::vector< fvec > samples;
	ipair trajectory = {-1,-1};
	ivec labels;
	vector<fvec> samples;
	vector<int> flags;
	fvec lol;
	vector< ipair > sequences;

std::cout << "Im IN!!!" << std::endl; 
	int a = contours.size(); int l;
	//cout << "kol-vo trajectoriy "; cin >> a;
	for (int j = 0; j < a; j++){
		//cout << "kol-vo tochek " << j+1 << " traj ";
		//cin >> l;
        l = contours[j].size();
		for (int i = 0; i < l; i++){
			if (trajectory.first == -1) // we're starting a trajectory
				trajectory.first = samples.size();//canvas->data->GetCount();
			// we don't want to draw too often
			//if(drawTime.elapsed() < 50/speed) return; // msec elapsed since last drawing
			float x, y;
			//cout << i+1<<":" << endl << "x="; cin >> x; cout << "y=";cin >> y;
			x = contours[j][i].x;
            y = contours[j][i].y;
            lol = { x, y };
			AddSample(lol, samples);
			flags.push_back(2);
			labels.push_back(j);
			trajectory.second = samples.size() - 1;
		}

		if (trajectory.first != -1)
		{
			// the last point is a duplicate, we take it out
			AddSequence(trajectory, sequences);
			trajectory.first = -1;
			
		}
	}
	
	
	Dynamize(samples,  sequences, labels);
	//Train(dyn, samples, sequences);
	//Draw(knn);
	
//	system("PAUSE");
	return 0;
	
}

