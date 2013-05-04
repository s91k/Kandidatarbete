#include "AnnAIController.h"
#include "NeuralNetwork.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
AnnAIController::AnnAIController()
{
	//this->zCar = car;

	this->Init();
}

AnnAIController::~AnnAIController()
{

}

void AnnAIController::Init()
{
	//Undecided amount
	this->zNumInputs = 10;
	this->zNumHiddenNodes = 1;
	this->zMaximumErrorAllowed = 0.005f;
	this->zNumSavedTrainingSets = 0;

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

bool AnnAIController::Update()
{
	static int index = 0;
	Car* tempCar = NULL;
	float temp = 0.5f;

	switch (this->zNetMode)
	{
	case NN_TRAIN:
		FILE* pfile;
		pfile = fopen("NeuralNetworkTrainingData.txt", "a");
		if (pfile == NULL)
			return true;
		
		zNumSavedTrainingSets++;
		tempCar = this->zTrainingData[index];
		this->zInputs.push_back(tempCar->speed);
		this->zInputs.push_back(tempCar->angle);
		this->zInputs.push_back(tempCar->distR);
		this->zInputs.push_back(tempCar->distFR);
		this->zInputs.push_back(tempCar->distFFR);
		this->zInputs.push_back(tempCar->distF);
		this->zInputs.push_back(tempCar->distL);
		this->zInputs.push_back(tempCar->distFL);
		this->zInputs.push_back(tempCar->distFFL);
		this->zInputs.push_back(tempCar->clutch);

		//Final Value for accel/brake, clamped between 0.0 & 1.0
		//temp < 0.5 = brake else accel
		//temp * 2.0f - 1.0f to get correct value + * -1 for brake
		if (tempCar->accel > 0.0f)
		{
			temp = tempCar->accel;
			//Clamp value between 0.5 & 1.0
			temp = 0.5f * (temp + 1.0f);
			this->zOutputs.push_back(temp);
		}
		else if (tempCar->brake > 0.0f)
		{
			temp = tempCar->brake;
			//Make value negative so it ends up between 0 & 0.5
			temp = -temp;
			//Clamp value between 0.0 & 0.5
			temp = 0.5f * (temp + 1.0f);
			this->zOutputs.push_back(temp);
		}
		else
		{
			this->zOutputs.push_back(0.0f);
		}
		this->zOutputs.push_back(tempCar->steer);
		this->zOutputs.push_back(tempCar->gear);

		//Print inputs
		fprintf(pfile, "%f %f %f %f %f %f %f %f %f %f", 
			tempCar->speed, tempCar->angle, tempCar->distR,
			tempCar->distFR, tempCar->distFFR, tempCar->distF, 
			tempCar->distL, tempCar->distFL, tempCar->distFFL,
			tempCar->clutch
			);
		//Print outputs
		fprintf(pfile, "%f %f %f", temp, tempCar->steer, tempCar->gear);
		
		fclose(pfile);

		std::cout << index << "/" << this->zTrainingData.size() - 1 << " Lines read" << std::endl;
		
		index++;
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
		break;
	case NN_RETRAIN:
		return true;
		break;
	case NN_USE:
		return true;
	default:
		break;
	}
}

void AnnAIController::TrainNetAndSave()
{
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
			totalError = this->zNNetwork->GetError();
			//std::cout << totalError << std::endl;

		}
		totalError = this->zNNetwork->GetError();
		//std::cout << i << " / " << NUM_ITERATIONS_TO_TRAIN << " "<< totalError<<std::endl;
		//counter++;

		if (abs(totalError) < this->zMaximumErrorAllowed)
		{
			std::cout << "Total Error = " <<totalError << std::endl << std::endl;
			this->zNNetwork->WriteWeights();
			return;
		}
	}
		
	std::cout << "Total Error = " <<this->zNNetwork->GetError() << std::endl << std::endl;
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