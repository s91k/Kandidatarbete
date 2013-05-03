#pragma once

#include <vector>
#include "car.h"

class NeuralNetwork;

enum NEURAL_NET_TYPE
{
	NN_USE,
	NN_TRAIN,
	NN_RETRAIN
};

class AnnAIController
{
private:
	int zNumInputs;
	int zNumOutputs;
	int zNumHiddenLayers;
	int zNumHiddenNodes;
	int zNetMode;

	tCarElt* zCar;
	NeuralNetwork* zNNetwork;
	std::vector<float> zInputs;
	std::vector<float> zOutputs;
public:
	AnnAIController(tCarElt* car);
	virtual ~AnnAIController();

	void Init();
	void Reset();
	void Update();

	void TrainNetAndSave();

};