#include "AnnAIController.h"
#include "NeuralNetwork.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <windows.h>

AnnAIController::AnnAIController(TRAINING_TYPE type)
{
	std::srand((unsigned int)time(0));

	this->zTraining_type = type;

	this->Init();
}

AnnAIController::~AnnAIController()
{
	delete this->zNNetwork;
}

void AnnAIController::Init()
{
	this->zNumSavedTrainingSets = 0;

	this->zInputs.clear();
	this->zOutputs.clear();

	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		this->zNumInputs = 10;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 120;
		this->zMaximumErrorAllowed = 0.05f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	case TRAINING_TYPE_STEER:
		this->zNumInputs = 9;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 30;
		this->zMaximumErrorAllowed = 0.05f;
		this->zRequiredCorrectPercentage = 0.98f;
		break;
	case TRAINING_TYPE_GEAR:
		this->zNumInputs = 2;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 40;
		this->zMaximumErrorAllowed = 0.10f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	case TRAINING_TYPE_FULL:
	default:
		this->zNumInputs = 10;
		this->zNumOutputs = 3;
		this->zNumHiddenNodes = 480;
		this->zMaximumErrorAllowed = 0.15f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	}

	this->zNumHiddenLayers = 1;

	this->zNetMode = NN_USE;

	if (this->zNetMode == NN_USE)
	{
		std::string filename;

		switch (this->zTraining_type)
		{
		case TRAINING_TYPE_SPEED:
			filename = "NNSpeedWeightData.txt";
			break;
		case TRAINING_TYPE_STEER:
			filename = "NNSteerWeightData.txt";
			break;
		case TRAINING_TYPE_GEAR:
			filename = "NNGearWeightData.txt";
			break;
		case TRAINING_TYPE_FULL:
		default:
			filename = "NNWeightData.txt";
			break;
		}

		this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes, filename);
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
	std::string filename;
	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		filename = "NNSpeedWeightData.txt";
		break;
	case TRAINING_TYPE_STEER:
		filename = "NNSteerWeightData.txt";
		break;
	case TRAINING_TYPE_GEAR:
		filename = "NNGearWeightData.txt";
		break;
	case TRAINING_TYPE_FULL:
	default:
		filename = "NNWeightData.txt";
		break;
	}

	this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes, filename);

	bool TrainingVarsSet = false;
	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		this->zNNetwork->SetLearningRate(0.0009f);
		this->zNNetwork->SetMomentum(0.3f);
		TrainingVarsSet = true;
		break;
	case TRAINING_TYPE_STEER:
		this->zNNetwork->SetLearningRate(0.001f);
		this->zNNetwork->SetMomentum(0.6f);
		TrainingVarsSet = true;
		break;
	case TRAINING_TYPE_GEAR:
		this->zNNetwork->SetLearningRate(0.009f);
		this->zNNetwork->SetMomentum(0.8f);
		TrainingVarsSet = false;
		break;
	case TRAINING_TYPE_FULL:
	default:
		this->zNNetwork->SetLearningRate(0.009f);
		this->zNNetwork->SetMomentum(0.5f);
		TrainingVarsSet = false;
		break;
	}

	std::cout << "Running Training " << std::endl << std::endl;
	
	int nrOfIterations = 0;
	int lastIterationCnt = 0;
	float PrintTimer = 5.0f;
	
	float CurrError = 0.0f;

	bool trained = false;
	float percentageCorrect = 0.0f;
	//Extra iterations
	int nrOfTestIter = 2;

	//used to get Correct percentage
	int numCorrect = 0;
	int numTotal = 0;

	float ErrorDifference = 0.0f;
	float CurrAvgError = 0.0f;
	float PrevAvgError = 0.0f;
	float PrevError = 0.0f;

	int nrOfDec = 0;
	int nrOfInc = 0;
	int nrOfEqual = 0;

	float CurrentLearningRate = this->zNNetwork->GetLearningRate();

	//Performance Variables
	INT64 Frequency = 0;
	INT64 StartTime = 0;
	INT64 CurrentTime = 0;
	float DeltaTime = 0.0f;
	float SecsPerCnt = 0.0f;
	float TimeDifference = 0.0f;
	float RunTime = 0.0f;
	float TotalRunTime = 0.0f;

	QueryPerformanceFrequency( (LARGE_INTEGER*) &Frequency);

	SecsPerCnt = 1.0f / (float)(Frequency);

	QueryPerformanceCounter( (LARGE_INTEGER*) &StartTime);

	std::vector<float> tempIns;
	std::vector<float> tempOuts;
	//Loop through num iterations
	while (!trained)
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

		nrOfIterations++;

		//Do a test to see how good the network is
		CurrError = 0.0f;
		numCorrect = 0;
		numTotal = 0;
		for (int u = 0; u < nrOfTestIter; u++)
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

				std::vector<float> networkOutput;

				this->zNNetwork->Use(tempIns, networkOutput);

				for(unsigned int k = 0; k < tempOuts.size(); k++)
				{
					float temp = networkOutput[k] - tempOuts[k];
					if (abs(temp) <= this->zMaximumErrorAllowed)
						numCorrect++;

					numTotal++;
					CurrError += abs(networkOutput[k] - tempOuts[k]);
				}
			}
		}
		
		//Update time
		QueryPerformanceCounter( (LARGE_INTEGER*) &CurrentTime);

		TimeDifference = (float) (CurrentTime - StartTime);
		
		DeltaTime = TimeDifference * SecsPerCnt;
		RunTime += DeltaTime;
		TotalRunTime += DeltaTime;

		StartTime = CurrentTime;

		percentageCorrect = ((float)numCorrect / (float)numTotal);
		
		if (nrOfIterations > 1)
		{
			if (CurrError < PrevError)
				nrOfDec++;
			else if(CurrError > PrevError)
				nrOfInc++;
			else
				nrOfEqual++;

			if (!TrainingVarsSet )
			{
				if (CurrError > PrevError)
				{
					CurrentLearningRate = this->zNNetwork->GetLearningRate();
					this->zNNetwork->SetLearningRate(CurrentLearningRate + (CurrentLearningRate * 0.001f) );
				}
				else if (CurrError < PrevError)
				{
					CurrentLearningRate = this->zNNetwork->GetLearningRate();
					this->zNNetwork->SetLearningRate(CurrentLearningRate + (CurrentLearningRate * 0.001f) );
				}
			}
		}

		if (RunTime >= PrintTimer)
		{
			PrevAvgError = CurrAvgError;
			CurrAvgError = CurrError / (this->zNumSavedTrainingSets * nrOfTestIter);

			std::cout 
				<< nrOfIterations - lastIterationCnt << " iterations in " << RunTime << "s" << std::endl
				<< nrOfIterations << " Total iterations (" << TotalRunTime << "s)"<< std::endl
				<< nrOfDec + nrOfInc + nrOfEqual<< " Total errors recorded \n"  
				<< nrOfDec << " Times error decreased \n" 
				<< nrOfInc << " Times error Increased \n"
				<< nrOfEqual << " Times error didn't change: \n"
				<< "Current Learning rate " << CurrentLearningRate << std::endl
				<< "Correct output "	<< percentageCorrect << "%\n"
				<< "Average Error " << CurrAvgError 
				<< " (" << CurrAvgError - PrevAvgError<< ")"
				<< std::endl << std::endl;

			lastIterationCnt = nrOfIterations;
			RunTime = 0;
		}
		PrevError = CurrError;
		trained = percentageCorrect >= this->zRequiredCorrectPercentage; 
		/*(abs(totalError) < this->zMaximumErrorAllowed * this->zNumSavedTrainingSets * nrOfTestIter);*/
	}

	this->zFinalError = CurrError;

	this->zNNetwork->WriteWeights();
	
	std::cout << nrOfIterations << " Total iterations (" << TotalRunTime << "s)" << std::endl
		<< "Current Learning rate " << CurrentLearningRate << std::endl
		<< "Correct output "	<< percentageCorrect << "%" << std::endl
		<< "Final Learning Rate " << this->zNNetwork->GetLearningRate() << std::endl
		<< "Average Error " << CurrError / (this->zNumSavedTrainingSets * 2) << std::endl
		<< "Writing Weights to file" << std::endl
		<< std::endl << std::endl;
}

void AnnAIController::RunTraining( std::vector<Car*> trainingData )
{
	Car* car;

	auto it_end = trainingData.cend();
	for (auto it_data = trainingData.cbegin(); it_data != it_end; it_data++)
	{
		this->zNumSavedTrainingSets++;
		car = (*it_data);

		float temp = 0.5f;

		switch (this->zTraining_type)
		{
		case TRAINING_TYPE_SPEED:
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

			if (car->accel > 0.0f)
				temp = car->accel;
			else if(car->brake > 0.0f)
				temp = -car->brake;
			else
				temp = 0.5f;

			temp = 0.5f * (temp + 1.0f);			

			this->zOutputs.push_back(temp);
			break;
		case TRAINING_TYPE_STEER:
			this->zInputs.push_back(car->speed);
			this->zInputs.push_back(car->angle);
			this->zInputs.push_back(car->distR);
			this->zInputs.push_back(car->distFR);
			this->zInputs.push_back(car->distFFR);
			this->zInputs.push_back(car->distF);
			this->zInputs.push_back(car->distL);
			this->zInputs.push_back(car->distFL);
			this->zInputs.push_back(car->distFFL);

			temp = 0.5f * (car->steer + 1.0f);
			this->zOutputs.push_back(temp);
			break;
		case TRAINING_TYPE_GEAR:
			this->zInputs.push_back(car->speed);
			this->zInputs.push_back(car->clutch);

			temp = car->gear / 6;
			temp = 0.5f * (temp + 1.0f);
			this->zOutputs.push_back(temp);
			break;
		case TRAINING_TYPE_FULL:
		default:
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

			if (car->accel > 0.0f)
				temp = car->accel;
			else if(car->brake > 0.0f)
				temp = -car->brake;	
			else
				temp = 0.5f;

			temp = 0.5f * (temp + 1.0f);

			this->zOutputs.push_back(temp);

			temp = 0.5f * (car->steer + 1.0f);
			this->zOutputs.push_back(temp);

			temp = car->gear / 6;
			temp = 0.5f * (temp + 1.0f);
			this->zOutputs.push_back(temp);
			break;
		}
	}
	
	std::cout << "Training data input done" <<std::endl;

	std::cout << "Beginning training network " <<std::endl;
	TrainNetAndSave();
	return;
}

void AnnAIController::Run(Car *car)
{
	std::vector<float> inputs;
	std::vector<float> outputs;

	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		inputs.push_back(car->speed);
		inputs.push_back(car->angle);
		inputs.push_back(car->clutch);
		break;
	case TRAINING_TYPE_STEER:
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
		break;
	case TRAINING_TYPE_GEAR:
		inputs.push_back(car->speed);
		break;
	case TRAINING_TYPE_FULL:
	default:
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
		break;
	}

	this->zNNetwork->Use(inputs, outputs);

	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		car->brake = 0.0f;
		car->accel = 0.0f;

		if (outputs[0] > 0.5f)
			car->accel = (2.0f * outputs[0] - 1.0f);
		else if (outputs[0] < 0.5f)
			car->brake = (2.0f * outputs[0] - 1.0f);

		break;
	case TRAINING_TYPE_STEER:
		car->steer = (2.0f * outputs[0] - 1.0f);
		break;
	case TRAINING_TYPE_GEAR:
		outputs[0] *= 6;
		car->gear = (2.0f * outputs[0] - 1.0f);
		break;
	case TRAINING_TYPE_FULL:
	default:
		car->brake = 0.0f;
		car->accel = 0.0f;

		if (outputs[0] > 0.5f)
			car->accel = ( 2.0f * outputs[0] - 1.0f);
		else if (outputs[0] < 0.5f)
			car->brake = (2.0f * outputs[0] - 1.0f);

		car->steer = (2.0f * outputs[1] - 1.0f);

		outputs[2] *= 6;
		car->gear = (2.0f * outputs[2] - 1.0f);
		break;
	}
}