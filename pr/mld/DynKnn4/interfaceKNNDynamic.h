
/*********************************************************************/
#ifndef _INTERFACEKNNDYNAMIC_H_
#define _INTERFACEKNNDYNAMIC_H_

#include <vector>
//#include <interfaces.h>
#include "DynKnn.h"


class DynamicKNN {
	protected:
	public:
	public:
	DynamicKNN();
	// virtual functions to manage the algorithm creation
	Dynamical *GetDynamical();
	//void DrawInfo(Canvas *canvas, QPainter &painter, Dynamical *dynamical){}
	//void DrawModel(Canvas *canvas, QPainter &painter, Dynamical *dynamical){}

	// virtual functions to manage the GUI and I/O
	//QString GetName(){ return QString("K-Nearest Neighbours"); }
	//QString GetAlgoString(){ return GetName(); }
	//QString GetInfoFile(){ return "knn.html"; }
	//bool UsesDrawTimer(){ return true; }
	//QWidget *GetParameterWidget(){ return widget; }
	void SetParams(Dynamical *dynamical);
	//void SaveOptions(QSettings &settings);
	//bool LoadOptions(QSettings &settings);
	//void SaveParams(QTextStream &stream);
	//bool LoadParams(QString name, float value);
	//public slots:
	//void ChangeOptions();
};

#endif // _INTERFACEKNNDYNAMIC_H_
