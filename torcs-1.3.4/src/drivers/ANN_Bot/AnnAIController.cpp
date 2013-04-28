#include "AnnAIController.h"

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

void AnnAIController::Update( float dt )
{
	switch (this->zNetMode)
	{
	case NN_TRAIN:

		break;
	case NN_RETRAIN:

		break;
	case NN_USE:

	default:
		break;
	}
}