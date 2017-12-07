#include <cstdlib>						// C standard lib defs
#include "ANN/ANNx.h"					// all ANN includes
#include "ANN/Annperf.h"		// ANN performance

using namespace std;					// make std:: accessible
ANN_METRIC ANN::MetricType = ANN_METRIC0;
double ANN::MetricPower = 3.;

//----------------------------------------------------------------------
//	Point methods
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	Distance utility.
//		(Note: In the nearest neighbor search, most distances are
//		computed using partial distance calculations, not this
//		procedure.)
//----------------------------------------------------------------------

ANNdist annDist(						// interpoint squared distance
	int					dim,
	ANNpoint			p,
	ANNpoint			q)
{
	register int d;
	register ANNcoord diff;
	register ANNcoord dist;

	dist = 0;
	for (d = 0; d < dim; d++) {
		diff = p[d] - q[d];
		switch (ANN::MetricType)
		{
		case ANN_METRIC0:
			dist = ANN_SUM0(dist, ANN_POW0(diff));
			break;
		case ANN_METRIC1:
			dist = ANN_SUM1(dist, ANN_POW1(diff));
			break;
		case ANN_METRIC2:
			dist = ANN_SUM(dist, ANN_POW(diff));
			break;
		case ANN_METRICP:
			dist = ANN_SUMp(dist, ANN_POWp(diff));
			break;
		}
	}
	ANN_FLOP(3 * dim)					// performance counts
		ANN_PTS(1)
		ANN_COORD(dim)
		return dist;
}

//----------------------------------------------------------------------
//	annPrintPoint() prints a point to a given output stream.
//----------------------------------------------------------------------

void annPrintPt(						// print a point
	ANNpoint			pt,				// the point
	int					dim,			// the dimension
	std::ostream		&out)			// output stream
{
	for (int j = 0; j < dim; j++) {
		out << pt[j];
		if (j < dim - 1) out << " ";
	}
}

//----------------------------------------------------------------------
//	Point allocation/deallocation:
//
//		Because points (somewhat like strings in C) are stored
//		as pointers.  Consequently, creating and destroying

//----------------------------------------------------------------------

ANNpoint annAllocPt(int dim, ANNcoord c)		// allocate 1 point
{
	ANNpoint p = new ANNcoord[dim];
	for (int i = 0; i < dim; i++) p[i] = c;
	return p;
}

ANNpointArray annAllocPts(int n, int dim)		// allocate n pts in dim
{
	ANNpointArray pa = new ANNpoint[n];			// allocate points
	ANNpoint	  p = new ANNcoord[n*dim];		// allocate space for coords
	for (int i = 0; i < n; i++) {
		pa[i] = &(p[i*dim]);
	}
	return pa;
}

void annDeallocPt(ANNpoint &p)					// deallocate 1 point
{
	delete[] p;
	p = NULL;
}

void annDeallocPts(ANNpointArray &pa)			// deallocate points
{
	delete[] pa[0];							// dealloc coordinate storage
	delete[] pa;								// dealloc points
	pa = NULL;
}

ANNpoint annCopyPt(int dim, ANNpoint source)	// copy point
{
	ANNpoint p = new ANNcoord[dim];
	for (int i = 0; i < dim; i++) p[i] = source[i];
	return p;
}

// assign one rect to another
void annAssignRect(int dim, ANNorthRect &dest, const ANNorthRect &source)
{
	for (int i = 0; i < dim; i++) {
		dest.lo[i] = source.lo[i];
		dest.hi[i] = source.hi[i];
	}
}

// is point inside rectangle?
ANNbool ANNorthRect::inside(int dim, ANNpoint p)
{
	for (int i = 0; i < dim; i++) {
		if (p[i] < lo[i] || p[i] > hi[i]) return ANNfalse;
	}
	return ANNtrue;
}

//----------------------------------------------------------------------
//	Error handler
//----------------------------------------------------------------------

void annError(const char* msg, ANNerr level)
{
	if (level == ANNabort) {
		cerr << "ANN: ERROR------->" << msg << "<-------------ERROR\n";
		exit(1);
	}
	else {
		cerr << "ANN: WARNING----->" << msg << "<-------------WARNING\n";
	}
}

//----------------------------------------------------------------------
//	Limit on number of points visited
//		We have an option for terminating the search early if the
//		number of points visited exceeds some threshold.  If the
//		threshold is 0 (its default)  this means there is no limit
//		and the algorithm applies its normal termination condition.
//		This is for applications where there are real time constraints
//		on the running time of the algorithm.
//----------------------------------------------------------------------

int	ANNmaxPtsVisited = 0;	// maximum number of pts visited
int	ANNptsVisited;			// number of pts visited in search

//----------------------------------------------------------------------
//	Global function declarations
//----------------------------------------------------------------------

void annMaxPtsVisit(			// set limit on max. pts to visit in search
	int					maxPts)			// the limit
{
	ANNmaxPtsVisited = maxPts;
}
