
#ifndef _DYNAMICAL_H_
#define _DYNAMICAL_H_
#include <vector>
#include <opencv2/highgui/highgui.hpp>

typedef float f32;
typedef std::vector<float> fvec;
typedef std::vector<int> ivec;
typedef int s32;
typedef unsigned int u32;
typedef std::pair<int, int> ipair;
//extern
enum dynamicalType { DYN_SVR, DYN_RVM, DYN_GMR, DYN_GPR, DYN_KNN, DYN_MLP, DYN_LINEAR, DYN_LWPR, DYN_KRLS, DYN_NONE };

class Dynamical
{
protected:
	std::vector< std::vector<fvec> > trajectories;
	ivec classes;
	ivec labels;
	u32 dim;

public:
	std::vector<fvec> crossval;
	fvec fmeasures;
	fvec trainErrors, testErrors;
	int type;
	float dT;
	u32 count;

	Dynamical();
	~Dynamical();
	std::vector< std::vector<fvec> > GetTrajectories(){ return trajectories; };

	virtual void Train(std::vector< std::vector<fvec> > trajectories, ivec labels){};
	virtual std::vector<fvec> Test(const fvec &sample, const int count){ return std::vector<fvec>(); };
	virtual fvec Test(const fvec &sample){ return fvec(); };
	virtual void Draw(IplImage *display){};
	virtual char *GetInfoString(){ return NULL; };
};

#endif // _DYNAMICAL_H_
