#pragma once

#include <vector>
#include "NeuralNetwork.h"

#define NUM_ITERATIONS_TO_TRAIN 10000

struct Car
{
	Car()
	{
		speed = 0.0f;
		angle = 0.0f;
		targetAngle = 0.0f;
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
	//Inputs
	float speed;
	float angle;
	float targetAngle;
	float distR;
	float distFR;
	float distFFR;
	float distF;
	float distFFL;
	float distFL;
	float distL;
	float clutch;
	//Outputs
	float steer;
	float accel;
	float brake;
	float gear;
	
};

enum NEURAL_NET_TYPE
{
	NN_USE,
	NN_TRAIN,
	NN_RETRAIN
};

enum TRAINING_TYPE
{
	TRAINING_TYPE_FULL, //Use 4 outputs
	TRAINING_TYPE_SPEED, // Use 2 outputs Accel & Brake
	TRAINING_TYPE_STEER, // Use 1 output Steering
	TRAINING_TYPE_GEAR // Use 1 output Gear
};

class AnnAIController
{
private:
	int zNumInputs;
	int zNumOutputs;
	int zNumHiddenLayers;
	int zNumHiddenNodes;
	int zNetMode;

	float zRequiredCorrectPercentage;
	float zMaximumErrorAllowed;
	NeuralNetwork* zNNetwork;
	std::vector<float> zInputs;
	std::vector<float> zOutputs;

	int zNumSavedTrainingSets;
	TRAINING_TYPE zTraining_type;

	float zFinalError;

	float Round(float num, int precision);
public:
	AnnAIController(TRAINING_TYPE type = TRAINING_TYPE_FULL);
	virtual ~AnnAIController();

	void Init();
	void Reset();
	void RunTraining(std::vector<Car*> trainingData);
	void TrainNetAndSave();

	float GetFinalError() {return this->zFinalError;}

	void Run(Car *car);
	void ResetTraining();
	void PrintData(const std::string& info, const float& expectedOutput, const float& generatedOutput);
};