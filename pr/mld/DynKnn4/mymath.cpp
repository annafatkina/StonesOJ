#include "mymath.h"
#include "DynKnn4.h"
typedef std::vector<float> fvec;

void operator+= (fvec &a, const fvec b)
{
	if (a.size() == 2)
	{
		a[0] += b[0];
		a[1] += b[1];
	}
	else
	{
		for (int i = 0; i < min(a.size(), b.size());i++) a[i] += b[i];
	}
}

int min(int a, int b)
{
	if (a < b) return a; else return b;
}

void operator-= (fvec &a, const fvec b)
{
	if (a.size() == 2)
	{
		a[0] -= b[0];
		a[1] -= b[1];
	}
	else
	{
		for (int i = 0; i < min(a.size(), b.size()); i++) a[i] -= b[i];
	}
}
void operator+= (fvec &a, const float b)
{
	if (a.size() == 2)
	{
		a[0] += b;
		a[1] += b;
	}
	else
	{
		for (int i = 0; i < a.size(); i++) a[i] += b;
	}
}
void operator-= (fvec &a, const float b)
{
	if (a.size() == 2)
	{
		a[0] -= b;
		a[1] -= b;
	}
	else
	{
		for (int i = 0; i < a.size(); i++) a[i] -= b;
	}
}
void operator *= (fvec &a, const float b)
{
	if (a.size() == 2)
	{
		a[0] *= b;
		a[1] *= b;
	}
	else
	{
		for (int i = 0; i < a.size(); i++) a[i] *= b;
	}
}
void operator /= (fvec &a, const float b)
{
	if (a.size() == 2)
	{
		a[0] /= b;
		a[1] /= b;
	}
	else
	{
		for (int i = 0; i < a.size(); i++) a[i] /= b;
	}
}

fvec operator + (const fvec a, const fvec b)
{
	fvec c = a;
	for (int i = 0; i < c.size(); i++) c[i] += b[i];
	return c;
}
fvec operator - (const fvec a, const fvec b)
{
	fvec c = a;
	for (int i = 0; i < a.size(); i++) c[i] -= b[i];
	return c;
}
fvec operator + (const fvec a, const float b)
{
	fvec c = a;
	for (int i = 0; i < c.size(); i++) c[i] += b;
	return c;
}
fvec operator - (const fvec a, const float b)
{
	fvec c = a;
	for (int i = 0; i < c.size(); i++) c[i] -= b;
	return c;
}
fvec operator * (const fvec a, const float b)
{
	fvec c = a;
	for (int i = 0; i < c.size(); i++) c[i] *= b;
	return c;
}
fvec operator / (const fvec a, const float b)
{
	fvec c = a;
	for (int i = 0; i < c.size(); i++) c[i] /= b;
	return c;
}

float operator * (const fvec a, const fvec b)
{
	float sum = 0;
	for (int i = 0; i < a.size(); i++) sum += a[i] * b[i];
	return sum;
}

std::vector<fvec> interpolate(std::vector<fvec> a, int count)
{
	// basic interpolation
	std::vector<fvec> res;
	res.resize(count);
	for(int i = 0; i < count;i++)
	{
		int length = a.size();
		float ratio = i / (float)count;
		int index = (int)(ratio*length);
		float remainder = ratio*length - (float)(int)(ratio*length);
		if (remainder == 0 || index == length - 1) res[i] = a[index];
		else // we need to interpolate
		{
			fvec pt0 = a[index];
			fvec pt1 = a[index + 1];
			res[i] = pt0*(1.f - remainder) + pt1*remainder;
		}
	}
	return res;
}
