#include "AnnAIController.h"
#include "NeuralNetwork.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>

AnnAIController::AnnAIController()
{
	std::srand(time(0));

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
	this->zMaximumErrorAllowed = 1.0f;
	this->zNumSavedTrainingSets = 0;

	this->zInputs.clear();
	this->zOutputs.clear();

	this->zNumOutputs = 4;
	this->zNumHiddenLayers = 1;

	this->zNetMode = NN_USE;

	if (this->zNetMode == NN_USE)
	{
		this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes);
		this->zNNetwork->ReadWeights();
	}
}

void AnnAIController::Reset()
{
	this->Init();
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
	for (int i = 0; /*i < NUM_ITERATIONS_TO_TRAIN*/ true; i++)
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


		//Do a test to see how good the network is
		totalError = 0.0f;

		for(int j = 0; j < 10; j++)
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

			std::vector<float> networkOutput;

			this->zNNetwork->Use(tempIns, networkOutput);

			for(int k = 0; k < tempOuts.size(); k++)
			{
				totalError += abs(networkOutput[k] - tempOuts[k]);
			}
		}

		if (abs(totalError) < lowestAbsError)
		{
			lowestAbsError = abs(totalError);
			lowestError = totalError;
			std::cout << totalError << std::endl;
		}

		if (abs(totalError) < this->zMaximumErrorAllowed * 10)
		{
			std::cout << "Total Error = " <<totalError << std::endl;
			std::cout << "Writing weights to file " << std::endl;
			this->zNNetwork->WriteWeights();
			return;
		}
	}
		
	std::cout << "Total Error = " << totalError << std::endl <<
		"Lowest Abs error = "<< lowestAbsError<< std::endl <<
		"Lowest error = " << lowestError << std::endl;
}

bool AnnAIController::LoadTrainingData( std::string filename )
{
	std::fstream read;

	read.open(filename);

	if (!read.is_open())
		return false;

	Car tempCarData;
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

			tempCarData = Car();
			first = false;
		}
		else if (dataType == "speed")
		{
			tempCarData.speed = floatVal;
		}
		else if (dataType == "angle")
		{
			tempCarData.angle = floatVal;
		}
		else if (dataType == "distR")
		{
			tempCarData.distR = floatVal;
		}
		else if (dataType == "distFR")
		{
			tempCarData.distFR = floatVal;
		}
		else if (dataType == "distFFR")
		{
			tempCarData.distFFR = floatVal;
		}
		else if (dataType == "distF")
		{
			tempCarData.distF = floatVal;
		}
		else if (dataType == "distFFL")
		{
			tempCarData.distFFL = floatVal;
		}
		else if (dataType == "distFL")
		{
			tempCarData.distFL = floatVal;
		}
		else if (dataType == "distL")
		{
			tempCarData.distL = floatVal;
		}
		else if (dataType == "steer")
		{
			tempCarData.steer = floatVal;
		}
		else if (dataType == "accel")
		{
			tempCarData.accel = floatVal;
		}
		else if (dataType == "brake")
		{
			tempCarData.brake = floatVal;
		}
		else if (dataType == "gear")
		{
			tempCarData.gear = floatVal;
		}
		else if (dataType == "clutch")
		{
			tempCarData.clutch = floatVal;
		}
	}

	this->zTrainingData.push_back(tempCarData);
	return true;
}

bool AnnAIController::RunTraining()
{
	static int index = 0;
	Car car;

	this->zNumSavedTrainingSets++;
	car = this->zTrainingData[index++];

	float temp = 0.5f;

	this->zInputs.push_back(car.speed);
	this->zInputs.push_back(car.angle);
	this->zInputs.push_back(car.distR);
	this->zInputs.push_back(car.distFR);
	this->zInputs.push_back(car.distFFR);
	this->zInputs.push_back(car.distF);
	this->zInputs.push_back(car.distL);
	this->zInputs.push_back(car.distFL);
	this->zInputs.push_back(car.distFFL);
	this->zInputs.push_back(car.clutch);

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
	this->zOutputs.push_back(car.accel);
	this->zOutputs.push_back(car.brake);
	this->zOutputs.push_back(car.steer);
	this->zOutputs.push_back(car.gear);


	FILE* pfile;
	pfile = fopen("NeuralNetworkTrainingData.txt", "a");
	if (pfile == NULL)
		return true;

	//Print inputs
	fprintf(pfile, "%f %f %f %f %f %f %f %f %f %f\n", 
		car.speed, car.angle, car.distR,
		car.distFR, car.distFFR, car.distF, 
		car.distL, car.distFL, car.distFFL,
		car.clutch
		);

	//Print outputs
	fprintf(pfile, "%f %f %f %f\n", car.accel, car.brake, car.steer, car.gear);

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

void AnnAIController::run(Car *car)
{
	std::vector<float> inputs;
	std::vector<float> outputs;

	inputs.push_back(car->speed);
	inputs.push_back(car->angle);
	inputs.push_back(car->distR);
	inputs.push_back(car->distFR);
	inputs.push_back(car->distFFR);
	inputs.push_back(car->distF);
	inputs.push_back(car->distL);
	inputs.push_back(car->distFL);
	inputs.push_back(car->distFFL);
	inputs.push_back(car->clutch);	

	this->zNNetwork->Use(inputs, outputs);

	// Accel/Brake
	//float value = outputs[0];
	//if (value < 0.5f) //Brake
	//{
	//	value = 2.0f * value - 1.0f;
	//	//Set Brake Value
	//	car->brake = value;

	//}
	//else if (value > 0.5f) //Accel
	//{
	//	value = 2.0f * value - 1.0f;
	//	//Set Accel to Value
	//	car->accel = value;
	//}
	//else // Both = 0.0
	//{
	//	//Set Both Values to 0.0
	//	car->accel = 0.0f;
	//	car->brake = 0.0f;
	car->accel = outputs[0];
	car->brake = outputs[1];
	//}
	// Steer
	car->steer = outputs[2];
	// Gear
	car->gear = outputs[3];
}