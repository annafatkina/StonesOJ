
#ifndef _BASICMATH_H_
#define _BASICMATH_H_


#include <vector>


typedef float f32;
typedef std::vector<float> fvec;


static std::vector<float> Quartiles(std::vector<float> srcdata)
{
	std::vector<float> quartiles;
	if (!srcdata.size())
	{
		quartiles.resize(5, 0);
		return quartiles;
	}
	// we take out all non-zero elements
	std::vector<float> data;
	for (int i = 0; i < srcdata.size(); i++) if (srcdata[i] != 0) data.push_back(srcdata[i]);
	if (!data.size())
	{
		quartiles.resize(5, 0);
		return quartiles;
	}
	if (data.size() == 1)
	{
		quartiles.resize(5, data[0]);
		return quartiles;
	}
	float mean = 0;
	float sigma = 0;
	for (int i = 0; i < data.size(); i++) mean += data[i] / data.size();
	for (int i = 0; i < data.size(); i++) sigma += powf(data[i] - mean, 2);
	sigma = sqrtf(sigma / data.size());

	std::vector<float> outliers;
	std::vector<float> sorted;
	if (sigma == 0)
	{
		sorted = data;
	}
	else
	{
		// we look for outliers using the 3*sigma rule
		for (int i = 0; i < data.size(); i++)
		{
			if (data[i] - mean < 3 * sigma)
				sorted.push_back(data[i]);
			else outliers.push_back(data[i]);
		}
	}
	if (!sorted.size())
	{
		quartiles.resize(5, 0);
		return quartiles;
	}
	sort(sorted.begin(), sorted.end());
	int count = sorted.size();
	int half = count / 2;
	float bottom = sorted[0];
	float top = sorted[sorted.size() - 1];

	float median = count % 2 ? sorted[half] : (sorted[half] + sorted[half - 1]) / 2;
	float quartLow, quartHi;
	if (count < 4)
	{
		quartLow = bottom;
		quartHi = top;
	}
	else
	{
		quartLow = half % 2 ? sorted[half / 2] : (sorted[half / 2] + sorted[half / 2 - 1]) / 2;
		quartHi = half % 2 ? sorted[half * 3 / 2] : (sorted[half * 3 / 2] + sorted[half * 3 / 2 - 1]) / 2;
	}
	quartiles.push_back(bottom);
	quartiles.push_back(quartLow);
	quartiles.push_back(median);
	quartiles.push_back(quartHi);
	quartiles.push_back(top);
	return quartiles;
}

static std::vector<float> MeanStd(std::vector<float> srcdata)
{
	std::vector<float> results;
	results.resize(2, 0);
	if (!srcdata.size())
	{
		return results;
	}
	if (srcdata.size() == 1)
	{
		results[0] = srcdata[0];
		results[1] = 0;
		return results;
	}
	float mean = 0;
	float stdev = 0;
	for (int i = 0; i < srcdata.size(); i++)
	{
		mean += srcdata[i];
	}
	mean /= srcdata.size();
	for (int i = 0; i < srcdata.size(); i++)
	{
		stdev += (srcdata[i] - mean)*(srcdata[i] - mean);
	}
	stdev = sqrtf(stdev);
	results[0] = mean;
	results[1] = stdev;
	return results;
}


#endif // _BASICMATH_H_

