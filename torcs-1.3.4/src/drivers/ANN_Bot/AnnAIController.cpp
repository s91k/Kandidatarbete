#include "AnnAIController.h"
#include "NeuralNetwork.h"

AnnAIController::AnnAIController(tCarElt* car)
{
	this->zCar = car;

	this->Init();
}

AnnAIController::~AnnAIController()
{

}

void AnnAIController::Init()
{
	//Undecided amount
	this->zNumInputs = 0;
	this->zNumHiddenNodes = 0;

	this->zInputs.clear();
	this->zOutputs.clear();

	this->zNumOutputs = 3;
	this->zNumHiddenLayers = 1;

	this->zNetMode = NN_TRAIN;
}

void AnnAIController::Reset()
{
	this->Init();
}

void AnnAIController::Update()
{
	switch (this->zNetMode)
	{
	case NN_TRAIN:
		FILE* pfile;
		pfile = fopen("NeuralNetworkTrainingData.txt", "a");
		if (pfile == NULL)
			return;

		

		break;
	case NN_RETRAIN:

		break;
	case NN_USE:
		
	default:
		break;
	}
}

void AnnAIController::TrainNetAndSave()
{
	this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes);

	std::vector<float> tempIns;
	std::vector<float> tempOuts;
	//Loop through num iterations

		//Loop through num saved trainings
		tempIns.clear();
		tempOuts.clear();

		this->zNNetwork->Train(tempIns, tempOuts);

	float totalError = this->zNNetwork->GetError();
}