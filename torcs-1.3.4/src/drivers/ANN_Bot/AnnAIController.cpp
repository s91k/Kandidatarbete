#include "AnnAIController.h"
#include "NeuralNetwork.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
//#include <time.h>

AnnAIController::AnnAIController(tCarCtrl* car)
{
	this->zCar = car;

	//std::srand(time(0));

	this->Init();
}

AnnAIController::~AnnAIController()
{

}

void AnnAIController::Init()
{
	//Undecided amount
	this->zNumInputs = 10;
	this->zNumHiddenNodes = 25;
	this->zMaximumErrorAllowed = 0.02f;
	this->zNumSavedTrainingSets = 0;

	this->zInputs.clear();
	this->zOutputs.clear();

	this->zNumOutputs = 4;
	this->zNumHiddenLayers = 1;

	this->zNetMode = NN_TRAIN;

	if (this->zNetMode == NN_USE)
	{
		this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes);
		this->zNNetwork->ReadWeights();
	}
	//else if (this->zNetMode == NN_TRAIN)
	//{
	//	this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes);
	//	this->zNNetwork->ReadWeights();
	//}
}

void AnnAIController::Reset()
{
	this->Init();
}

bool AnnAIController::Update()
{
	switch (this->zNetMode)
	{
	case NN_TRAIN:
		return this->RunTraining();

		break;
	case NN_RETRAIN:
		return true;
		break;
	case NN_USE:
	default:
		GetNetOutput();
		return true;
		break;
	}
	return true;
}

void AnnAIController::GetNetOutput()
{
	this->zInputs.clear();
	this->zOutputs.clear();

	Car* car = this->zTrainingData[25];
	//Set Inputs
	
	this->zInputs.push_back(car->speed);
	this->zInputs.push_back(car->angle);
	this->zInputs.push_back(car->distR);
	this->zInputs.push_back(car->distFR);
	this->zInputs.push_back(car->distFFR);
	this->zInputs.push_back(car->distF);
	this->zInputs.push_back(car->distL);
	this->zInputs.push_back(car->distFL);
	this->zInputs.push_back(car->distFFL);
	this->zInputs.push_back(car->clutch);
	
	this->zNNetwork->Use(this->zInputs, this->zOutputs);

	// Accel/Brake
	//float value = this->zOutputs[0];
	//if (value < 0.5f) //Brake
	//{
	//	value = 2.0f * value - 1.0f;
	//	//Set Brake Value

	//}
	//else if (value > 0.5f) //Accel
	//{
	//	value = 2.0f * value - 1.0f;
	//	//Set Accel to Value
	//}
	//else // Both = 0.0
	//{
	//	//Set Both Values to 0.0

	//}
	this->zCar->accelCmd = this->zOutputs[0];
	this->zCar->brakeCmd = this->zOutputs[1];
	// Steer
	this->zCar->steer = this->zOutputs[2];
	// Gear
	this->zCar->gear = this->zOutputs[3];
}

void AnnAIController::TrainNetAndSave()
{
	//Stats
	float lowestAbsError = 9999999.9f;
	float lowestError = lowestAbsError;

	this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes);

	std::vector<float> tempIns;
	std::vector<float> tempOuts;
	float totalError = 0.0f;
	//int counter = 0;
	//int totalIterations = NUM_ITERATIONS_TO_TRAIN * this->zNumSavedTrainingSets;
	//Loop through num iterations
	for (int i = 0; i < NUM_ITERATIONS_TO_TRAIN; i++)
	{
		for (int j = 0; j < this->zNumSavedTrainingSets; j++)
		{
			//Loop through num saved trainings
			tempIns.clear();
			tempOuts.clear();
			//Get Training sets inputs
			for (int k = 0; k < this->zNumInputs; k++)
				tempIns.push_back(this->zInputs[k + j * this->zNumInputs]);

			//Get Training sets outputs
			for (int k = 0; k < this->zNumOutputs; k++)
				tempOuts.push_back(this->zOutputs[k + j * this->zNumOutputs]);

			this->zNNetwork->Train(tempIns, tempOuts);
		}

		totalError = this->zNNetwork->GetError();

		if (abs(totalError) < lowestAbsError)
		{
			lowestAbsError = abs(totalError);
			lowestError = totalError;
			std::cout << totalError << std::endl;
		}

		if (abs(totalError) < this->zMaximumErrorAllowed)
		{
			std::cout << "Total Error = " <<totalError << std::endl;
			std::cout << "Writing weights to file " << std::endl;
			this->zNNetwork->WriteWeights();
			return;
		}
	}
		
	std::cout << "Total Error = " <<this->zNNetwork->GetError() << std::endl <<
		"Lowest Abs error = "<< lowestAbsError<< std::endl <<
		"Lowest error = " << lowestError << std::endl;
}

bool AnnAIController::LoadTrainingData( std::string filename )
{
	std::fstream read;

	read.open(filename);

	if (!read.is_open())
		return false;

	Car* tempCarData = NULL;
	bool first = true;
	int index = 0;
	while (!read.eof())
	{
		std::string line;
		std::getline(read, line);

		size_t spacePosition = line.find(" ");

		std::string dataType = line.substr(0, spacePosition);
		std::string value = line.substr(spacePosition + 1);
		float floatVal;
		std::stringstream ss;
		ss << value;
		ss >> floatVal;

		if (dataType == "DATA")
		{
			if (!first)
				this->zTrainingData.push_back(tempCarData);

			tempCarData = new Car();
			first = false;
		}
		else if (dataType == "speed")
		{
			tempCarData->speed = floatVal;
		}
		else if (dataType == "angle")
		{
			tempCarData->angle = floatVal;
		}
		else if (dataType == "distR")
		{
			tempCarData->distR = floatVal;
		}
		else if (dataType == "distFR")
		{
			tempCarData->distFR = floatVal;
		}
		else if (dataType == "distFFR")
		{
			tempCarData->distFFR = floatVal;
		}
		else if (dataType == "distF")
		{
			tempCarData->distF = floatVal;
		}
		else if (dataType == "distFFL")
		{
			tempCarData->distFFL = floatVal;
		}
		else if (dataType == "distFL")
		{
			tempCarData->distFL = floatVal;
		}
		else if (dataType == "distL")
		{
			tempCarData->distL = floatVal;
		}
		else if (dataType == "steer")
		{
			tempCarData->steer = floatVal;
		}
		else if (dataType == "accel")
		{
			tempCarData->accel = floatVal;
		}
		else if (dataType == "brake")
		{
			tempCarData->brake = floatVal;
		}
		else if (dataType == "gear")
		{
			tempCarData->gear = floatVal;
		}
		else if (dataType == "clutch")
		{
			tempCarData->clutch = floatVal;
		}
	}

	this->zTrainingData.push_back(tempCarData);
	return true;
}

bool AnnAIController::RunTraining()
{
	static int index = 0;
	Car* car = NULL;

	this->zNumSavedTrainingSets++;
	car = this->zTrainingData[index++];

	float temp = 0.5f;

	this->zInputs.push_back(car->speed);
	this->zInputs.push_back(car->angle);
	this->zInputs.push_back(car->distR);
	this->zInputs.push_back(car->distFR);
	this->zInputs.push_back(car->distFFR);
	this->zInputs.push_back(car->distF);
	this->zInputs.push_back(car->distL);
	this->zInputs.push_back(car->distFL);
	this->zInputs.push_back(car->distFFL);
	this->zInputs.push_back(car->clutch);

	//Final Value for accel/brake, clamped between -1.0 & 1.0
	//temp < 0.5 = brake else accel or 0.5 = 0.0 for both
	//if (car->accel > 0.0f)
	//{
	//	temp = car->accel;
	//	//Clamp value between 0.5 & 1.0
	//	//temp = 0.5f * (temp + 1.0f);
	//	this->zOutputs.push_back(temp);
	//}
	//else if (car->brake > 0.0f)
	//{
	//	temp = car->brake;
	//	//Make value negative so it ends up between 0 & 0.5
	//	temp = -temp;
	//	//Clamp value between 0.0 & 0.5
	//	//temp = 0.5f * (temp + 1.0f);
	//	this->zOutputs.push_back(temp);
	//}
	//else
	//{
	//	this->zOutputs.push_back(0.0f);
	//}
	this->zOutputs.push_back(car->accel);
	this->zOutputs.push_back(car->brake);
	this->zOutputs.push_back(car->steer);
	this->zOutputs.push_back(car->gear);


	FILE* pfile;
	pfile = fopen("NeuralNetworkTrainingData.txt", "a");
	if (pfile == NULL)
		return true;

	//Print inputs
	fprintf(pfile, "%f %f %f %f %f %f %f %f %f %f\n", 
		car->speed, car->angle, car->distR,
		car->distFR, car->distFFR, car->distF, 
		car->distL, car->distFL, car->distFFL,
		car->clutch
		);

	//Print outputs
	fprintf(pfile, "%f %f %f %f\n", car->accel, car->brake, car->steer, car->gear);

	fclose(pfile);

	if (this->zNumSavedTrainingSets == this->zTrainingData.size())
	{
		std::cout << "Training data input done" <<std::endl;

		std::cout << "Press Enter to begin training" <<std::endl;
		std::cin.ignore(1);

		std::cout << "Beginning training network " <<std::endl;
		TrainNetAndSave();
		return true;
	}

	return false;
}