#pragma once

#include <vector>
#include "NeuralNetwork.h"

#define NUM_ITERATIONS_TO_TRAIN 10

struct Car
{
	Car()
	{
		speed = 0.0f;
		angle = 0.0f;
		distR = 0.0f;
		distFR = 0.0f;
		distFFR = 0.0f;
		distF = 0.0f;
		distFFL = 0.0f;
		distFL = 0.0f;
		distL = 0.0f;
		steer = 0.0f;
		accel = 0.0f;
		brake = 0.0f;
		gear = 0.0f;
		clutch = 0.0f;
	}
	float speed;
	float angle;
	float distR;
	float distFR;
	float distFFR;
	float distF;
	float distFFL;
	float distFL;
	float distL;
	float steer;
	float accel;
	float brake;
	float gear;
	float clutch;
};

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

	float zMaximumErrorAllowed;
	NeuralNetwork* zNNetwork;
	std::vector<float> zInputs;
	std::vector<float> zOutputs;

	//Training data
	std::vector<Car*> zTrainingData;

	int zNumSavedTrainingSets;

public:
	AnnAIController();
	virtual ~AnnAIController();
	bool LoadTrainingData(std::string filename);

	void Init();
	void Reset();
	//Returns true if training is Done
	bool Update();
	bool RunTraining();
	void TrainNetAndSave();

};