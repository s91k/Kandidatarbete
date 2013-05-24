#include "AnnAIController.h"
#include "NeuralNetwork.h"
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <windows.h>

struct History
{
	float currError;
	float lowestError;
	float percentage;
	int nodes;
	int iterations;
};

AnnAIController::AnnAIController(TRAINING_TYPE type)
{
	std::srand((unsigned int)time(0));

	this->zTraining_type = type;
	this->zNNetwork = NULL;

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
		this->zNumInputs = 11;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 60;
		this->zNumHiddenLayers = 1;

		this->zMaximumErrorAllowed = 0.05f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	case TRAINING_TYPE_STEER:
		this->zNumInputs = 10;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 30;
		this->zNumHiddenLayers = 1;

		this->zMaximumErrorAllowed = 0.05f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	case TRAINING_TYPE_GEAR:
		this->zNumInputs = 2;
		this->zNumOutputs = 1;
		this->zNumHiddenNodes = 40;
		this->zNumHiddenLayers = 1;

		this->zMaximumErrorAllowed = 0.10f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	case TRAINING_TYPE_FULL:
	default:
		this->zNumInputs = 11;
		this->zNumOutputs = 3;
		this->zNumHiddenNodes = 140;
		this->zNumHiddenLayers = 1;

		this->zMaximumErrorAllowed = 0.1f;
		this->zRequiredCorrectPercentage = 0.95f;
		break;
	}

	this->zNetMode = NN_TRAIN;

	if (this->zNetMode == NN_USE)
	{
		std::string filename;

		switch (this->zTraining_type)
		{
		case TRAINING_TYPE_SPEED:
			filename = "NeuralWeights/NNSpeedWeightData.txt";
			break;
		case TRAINING_TYPE_STEER:
			filename = "NeuralWeights/NNSteerWeightData.txt";
			break;
		case TRAINING_TYPE_GEAR:
			filename = "NeuralWeights/NNGearWeightData.txt";
			break;
		case TRAINING_TYPE_FULL:
		default:
			filename = "NeuralWeights/NNWeightData.txt";
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

void AnnAIController::ResetTraining()
{
	std::string filename;
	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		filename = "NeuralWeights/NNSpeedWeightData.txt";
		break;
	case TRAINING_TYPE_STEER:
		filename = "NeuralWeights/NNSteerWeightData.txt";
		break;
	case TRAINING_TYPE_GEAR:
		filename = "NeuralWeights/NNGearWeightData.txt";
		break;
	case TRAINING_TYPE_FULL:
	default:
		filename = "NeuralWeights/NNWeightData.txt";
		break;
	}

	if (this->zNNetwork)
		delete this->zNNetwork;

	this->zNNetwork = new NeuralNetwork(this->zNumInputs, this->zNumOutputs, this->zNumHiddenLayers, this->zNumHiddenNodes, filename);

	switch (this->zTraining_type)
	{
	case TRAINING_TYPE_SPEED:
		this->zNNetwork->SetLearningRate(0.0009f);
		this->zNNetwork->SetMomentum(0.3f);
		break;
	case TRAINING_TYPE_STEER:
		this->zNNetwork->SetLearningRate(0.001f);
		this->zNNetwork->SetMomentum(0.6f);
		break;
	case TRAINING_TYPE_GEAR:
		this->zNNetwork->SetLearningRate(0.009f);
		this->zNNetwork->SetMomentum(0.8f);
		break;
	case TRAINING_TYPE_FULL:
	default:
		this->zNNetwork->SetLearningRate(0.001f);
		this->zNNetwork->SetMomentum(0.6f);
		break;
	}

	std::fstream write;

	write.open("Output History.txt", std::fstream::in | std::fstream::out | std::fstream::app);
	write << "Hidden Nodes: " << this->zNumHiddenNodes << "\n\n";

	write.close();
}

void AnnAIController::TrainNetAndSave()
{
	std::cout << "Running Training " << std::endl << std::endl;

	std::ofstream writeFile;
	writeFile.open ("Output History.txt", std::ios::out | std::ios::trunc);

	writeFile.close();

	this->ResetTraining();

	int nrOfIterations = 0;
	int lastIterationCnt = 0;
	int nrOfIterationsPerNode = 0;

	float PrintTimer = 10.0f;
	
	bool trained = false;
	float percentageCorrect = 0.0f;
	//Extra iterations
	int nrOfTestIter = 1;

	//used to get Correct percentage
	int numCorrect = 0;
	int numTotal = 0;

	float ErrorDifference = 0.0f;
	float CurrAvgError = 0.0f;
	float PrevAvgError = 0.0f;
	float CurrError = 0.0f;
	float LowestErr = 9999999.9f;
	int ErrsAboveMin = 0;
	std::vector<History*> errorHistory;
	History* currentData = new History();
	errorHistory.push_back(currentData);

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
	std::string output[] = {"Accel/Brake", "Steering", "Gear"};
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
		nrOfIterationsPerNode++;

		//Do a test to see how good the network is
		CurrError = 0.0f;
		numCorrect = 0;
		numTotal = 0;
		bool print = (int)(TotalRunTime) % 1000 == 0;
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

					if ( print )
						this->PrintData(output[k], tempOuts[k], networkOutput[k]);
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

		if (CurrError < LowestErr)
		{
			LowestErr = CurrError;
			ErrsAboveMin = 0;
		}
		else
		{
			ErrsAboveMin++;
		}

		if (ErrsAboveMin > 3000)
		{
			History* hist = new History();

			errorHistory.push_back(hist);
			currentData = hist;

			LowestErr = 9999999.9f;
			nrOfIterationsPerNode = ErrsAboveMin = 0;
			PrevAvgError = CurrError = 0.0f;
			this->zNumHiddenNodes += 5;
			this->ResetTraining();
		}

		if (RunTime >= PrintTimer)
		{
			PrevAvgError = CurrAvgError;
			CurrAvgError = CurrError / (this->zNumSavedTrainingSets * nrOfTestIter);

			currentData->lowestError = LowestErr / (float)(numTotal);
			currentData->percentage = percentageCorrect;
			currentData->nodes = this->zNumHiddenNodes;
			currentData->currError = CurrError / (float)(numTotal);
			currentData->iterations = nrOfIterationsPerNode;

			float hour, min, sec;

			float fracPart = modf( (TotalRunTime / 3600), &hour);
			fracPart = modf(fracPart * 60, &min);
			fracPart = modf(fracPart * 60, &sec);
			sec += fracPart;

			std::cout 
				//<< nrOfIterations - lastIterationCnt << " iterations in " << RunTime << "s" << std::endl
				<< nrOfIterations << " Total iterations (" << hour << "h " << min << "m " << sec << "s)"<< std::endl
				//<< nrOfDec << " Times error decreased \n" 
				//<< nrOfInc << " Times error Increased \n"
				//<< nrOfEqual << " Times error didn't change: \n"
				//<< "Lowest Error so far " << LowestErr << " Avg: (" << LowestErr / (this->zNumSavedTrainingSets * nrOfTestIter)<< ")"<< std::endl
				<< "Errors above lowest " << ErrsAboveMin << "/3000" << "\n"
				//<< "Current Hidden Nodes " << this->zNumHiddenNodes << std::endl
				//<< "Correct output "	<< percentageCorrect << "%\n"
				//<< "Average Error " << CurrAvgError 
				<< "Error change since last print: (" << Round((CurrAvgError - PrevAvgError) * (this->zNumSavedTrainingSets * nrOfTestIter), 4) << ") Total Error: "<<Round((CurrAvgError) * (this->zNumSavedTrainingSets * nrOfTestIter), 4) <<"\n\n";

				for (auto it_hist = errorHistory.cbegin(); it_hist != errorHistory.cend(); it_hist++)
				{
					std::cout << (*it_hist)->nodes << " Nodes. " << Round((*it_hist)->currError, 5) << " / " << Round((*it_hist)->lowestError, 5) 
						<< " Final/Lowest. " << Round((*it_hist)->percentage, 5) <<" Percent. " << (*it_hist)->iterations << " Iterations\n";
				}

				std::cout << std::endl << std::endl;

			lastIterationCnt = nrOfIterations;
			RunTime -= PrintTimer;
		}

		trained = percentageCorrect >= this->zRequiredCorrectPercentage; 
		/*(abs(totalError) < this->zMaximumErrorAllowed * this->zNumSavedTrainingSets * nrOfTestIter);*/
	}

	for (auto it_hist = errorHistory.cbegin(); it_hist != errorHistory.cend(); it_hist++)
	{
		History* tmp = (*it_hist);
		if (tmp)
			delete tmp;
	}
	errorHistory.clear();

	this->zFinalError = CurrError;

	this->zNNetwork->WriteWeights();
	
	std::cout << nrOfIterations << " Total iterations (" << TotalRunTime << "s)" << std::endl
		<< "Current Learning rate " << CurrentLearningRate << std::endl
		<< "Correct output "	<< percentageCorrect << "%" << std::endl
		<< "Final Learning Rate " << this->zNNetwork->GetLearningRate() << std::endl
		<< "Average Error " << CurrError / numTotal << std::endl
		<< "Writing Weights to file" << std::endl
		<< std::endl << std::endl;
}

float AnnAIController::Round(float num, int precision)
{
	return floorf(num * pow(10.0f,precision) + .5f)/pow(10.0f,precision);
}

void AnnAIController::PrintData( const std::string& info, const float& expectedOutput, const float& generatedOutput )
{
	std::fstream write;

	write.open("Output History.txt", std::fstream::in | std::fstream::out | std::fstream::app);
	write 
		<< info << "\n"
		<< "Generated Output: " << generatedOutput 
		<< ". Expected Output " << expectedOutput << "\n\n";

	write.close();
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
			this->zInputs.push_back(car->targetAngle);
			this->zInputs.push_back(car->distR);
			this->zInputs.push_back(car->distFR);
			this->zInputs.push_back(car->distFFR);
			this->zInputs.push_back(car->distF);
			this->zInputs.push_back(car->distL);
			this->zInputs.push_back(car->distFL);
			this->zInputs.push_back(car->distFFL);
			this->zInputs.push_back(car->clutch);

			if (car->accel > 0.0f)
				temp = min(car->accel, 1.0f);
			else if(car->brake > 0.0f)
				temp = -min(car->brake, 1.0f);	
			else
				temp = 0.5f;

			temp = 0.5f * (temp + 1.0f);			

			this->zOutputs.push_back(temp);
			break;
		case TRAINING_TYPE_STEER:
			this->zInputs.push_back(car->speed);
			this->zInputs.push_back(car->angle);
			this->zInputs.push_back(car->targetAngle);
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
			this->zInputs.push_back(car->targetAngle);
			this->zInputs.push_back(car->distR);
			this->zInputs.push_back(car->distFR);
			this->zInputs.push_back(car->distFFR);
			this->zInputs.push_back(car->distF);
			this->zInputs.push_back(car->distL);
			this->zInputs.push_back(car->distFL);
			this->zInputs.push_back(car->distFFL);
			this->zInputs.push_back(car->clutch);

			if (car->accel > 0.0f)
				temp = min(car->accel, 1.0f);
			else if(car->brake > 0.0f)
				temp = -min(car->brake, 1.0f);	
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
		inputs.push_back(car->targetAngle);
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
		car->gear = (2.0f * outputs[0] - 1.0f);
		car->gear *= 6;
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

		car->gear = (2.0f * outputs[2] - 1.0f);
		car->gear *= 6;
		break;
	}
}