
#include "ANN/ANn.h"
#include "ANN/ANNx.h"					// all ANN includes
#include "ANN/Annperf.h"
#include "DynKnn4.h"
#include "dynamical.h"
#include "mymath.h"
#include "DynKnn.h"
using namespace std;



void DynamicalKNN::Train(std::vector< std::vector<fvec> > trajectories, ivec labels)
{
	if (!trajectories.size()) return;
	int count = trajectories[0].size();
	if (!count) return;
	dim = trajectories[0][0].size() / 2;

	// we forget about time and just push in everything
	vector<fvec> samples;
	for (int i = 0; i < trajectories.size(); i++)
	{
		for (int j = 0; j < trajectories[i].size(); j++)
		{
			samples.push_back(trajectories[i][j]);
		}
	}
	if (!samples.size()) return;
	int sampleCount = samples.size();
	points.resize(sampleCount);
	velocities.resize(sampleCount);
	for (int i = 0; i < sampleCount; i++)
	{
		points[i].resize(dim);
		velocities[i].resize(dim);
		for (int d = 0; d < dim; d++)
		{
			points[i][d] = samples[i][d];
			velocities[i][d] = samples[i][dim + d];
		}
	}

	delete kdTreee;
	annClose();
	ANN::MetricType = (ANN_METRIC)metricType;
	ANN::MetricPower = metricP;

	dataPts = annAllocPts(sampleCount, dim);// allocate data points
	for (int i = 0; i < sampleCount; i++)
	{
		for (int d = 0; d < dim; d++) dataPts[i][d] = points[i][d];
	}
	kdTreee = new ANNkd_tree(dataPts, sampleCount, dim);
	cout << endl<<"DynamicalKNN::Train is over!"<<endl;
}

DynamicalKNN::~DynamicalKNN()
{
	//annClose();
	delete kdTreee;
}
std::vector<fvec> DynamicalKNN::Test(const fvec &sample, const int count)
{
	fvec start = sample;
	dim = sample.size();
	std::vector<fvec> res;
	res.resize(count);
	for (int i = 0; i < count; i++) res[i].resize(dim, 0);
	if (!points.size()) return res;
	fvec velocity; velocity.resize(dim, 0);
	for (int i = 0; i < count; i++)
	{
		res[i] = start;
		for (int j = 0; j < start.size(); j++) {
			//start[j] += velocity[j] * 0.05f;
			start[j] += velocity[j] * 0.5f;
		}
		velocity = Test(start);
	}
	return res;
}


fvec DynamicalKNN::Test(const fvec &sample)
{
	fvec res;
	res.resize(2, 0);
	int dim = sample.size();
	if (!points.size()) return res;
	double eps = 0; // error bound
	ANNpoint queryPt; // query point
	queryPt = annAllocPt(dim); // allocate query point
	ANNidxArray nnIdx = new ANNidx[k]; // allocate near neigh indices
	ANNdistArray dists = new ANNdist[k]; // allocate near neighbor dists
	for (int i = 0; i < dim; i++) queryPt[i] = sample[i];
	kdTreee->annkSearch(queryPt, k, nnIdx, dists, eps);

	float dsum = 0;
	vector<fvec> scores;
	scores.resize(k);
	for (int i = 0; i < k; i++)
	{
		if (nnIdx[i] >= points.size()) continue;
		if (dists[i] == 0) dsum += 0;
		else dsum += 1. / dists[i];
		scores[i].resize(dim);
		for (int d = 0; d < dim; d++) scores[i][d] = velocities[nnIdx[i]][d];
	}
	for (int i = 0; i < k; i++)
	{
		if (nnIdx[i] >= points.size()) continue;
		if (dists[i] == 0) continue;
		dists[i] = 1. / (dists[i]) / dsum;
	}

	fvec mean, stdev;
	mean.resize(dim, 0);
	stdev.resize(dim, 0);
	int cnt = 0;
	for (int i = 0; i < k; i++)
	{
		if (nnIdx[i] >= points.size()) continue;
		//mean += scores[i] / (scores.size());
		mean += scores[i] * dists[i];
		cnt++;
	}
	for (int i = 0; i < k; i++)
	{
		if (nnIdx[i] >= points.size()) continue;
		for (int d = 0; d < dim; d++)
			stdev[d] += (scores[i] - mean)[d] * (scores[i] - mean)[d];
	}
	for (int d = 0; d < dim; d++)
	{
		if (cnt) stdev[d] /= cnt;
		else stdev[d] = 0;
		stdev[d] = sqrtf(stdev[d]);
	}

	delete[] nnIdx; // clean things up
	delete[] dists;

	res = mean;
	//res[1] = stdev;

	return res;
}



void DynamicalKNN::Draw(IplImage *display)
{
}

void DynamicalKNN::SetParams(u32 k, int metricType, u32 metricP)
{
	this->k = k;
	switch (metricType)
	{
	case 0:
		this->metricType = ANN_METRIC1;
		this->metricP = 1;
		break;
	case 1:
		this->metricType = ANN_METRIC2;
		this->metricP = 2;
		break;
	case 2:
		this->metricType = ANN_METRICP;
		this->metricP = metricP;
		break;
	case 3:
		this->metricType = ANN_METRIC0;
		this->metricP = 0;
		break;
	}
}

char *DynamicalKNN::GetInfoString()
{
	stringstream out;
	out << "KNN\n";
	out << "K:" << k << "\n";
	out << "Metric: ";
	switch (metricType)
	{
	case 0:
		out << "infinite norm\n";
		break;
	case 1:
		out << "1-norm (Manhattan)\n";
		break;
	case 2:
		out << "2-norm (Euclidean)\n";
		break;
	case 3:
		out << metricP << "-norm\n";
		break;
	}
	auto result = out.str();
	cout << result << endl;
	char *text = new char[1024];
	return text;
}
