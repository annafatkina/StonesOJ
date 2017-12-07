
#include "interfaceKNNDynamic.h"


using namespace std;

DynamicKNN::DynamicKNN()
{
	//params = new Ui::ParametersKNNDynamic();
	//params->setupUi(widget = new QWidget());
	//connect(params->knnNormCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(ChangeOptions()));
	//ChangeOptions();
}
/*
void DynamicKNN::ChangeOptions()
{
	params->knnNormSpin->setVisible(params->knnNormCombo->currentIndex() == 2);
	params->labelPower->setVisible(params->knnNormCombo->currentIndex() == 2);
}
*/
void DynamicKNN::SetParams(Dynamical *dynamical)
{
	if (!dynamical) return;
	int k = 2;// params->knnKspin->value();
	int metricType = 1;// params->knnNormCombo->currentIndex();
	int metricP = 5;// params->knnNormSpin->value();

	((DynamicalKNN *)dynamical)->SetParams(k, metricType, metricP);
}

Dynamical *DynamicKNN::GetDynamical()
{
	DynamicalKNN *dynamical = new DynamicalKNN();
	SetParams(dynamical);
	return dynamical;
}

