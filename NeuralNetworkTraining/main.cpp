#include "AnnAIController.h"
#include "BackProp.H"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

bool loadCarData(const std::string& filename, std::vector<Car*>& data)
{
	std::fstream read;

	read.open(filename);

	if (!read.is_open())
		return false;

	std::cout << "Loading Training data from file " << filename << std::endl << std::endl;

	Car* tempCarData;
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
				data.push_back(tempCarData);

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
		else if (dataType == "targetAngle")
		{
			tempCarData->targetAngle = floatVal;
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

	data.push_back(tempCarData);
	return true;
}

float TrainAnnController(TRAINING_TYPE type, std::vector<Car*>& trainingData)
{
	AnnAIController* controller = new AnnAIController(type);

	std::string training = "";
	switch (type)
	{
	case TRAINING_TYPE_SPEED:
		training = "Speed";
		break;
	case TRAINING_TYPE_STEER:
		training = "Steering";
		break;
	case TRAINING_TYPE_GEAR:
		training = "Gear";
		break;
	case TRAINING_TYPE_FULL:
	default:
		training = "Full";
		break;
	}
	std::cout << "Running " << training << " Training" << std::endl << std::endl;

	controller->RunTraining(trainingData);

	float error = controller->GetFinalError();

	delete controller;

	return error;
}

void TrainBackProp(TRAINING_TYPE type, const std::string& filename)
{
	// defining a net with 4 layers having 3,3,3, and 1 neuron respectively,
	// the first layer is input layer i.e. simply holder for the input parameters
	// and has to be the same size as the no of input parameters, in out example 3
	int numLayers = 3;
	int layerSize[3] = {10, 30, 1};

	switch (type)
	{
	case TRAINING_TYPE_FULL:
		break;
	case TRAINING_TYPE_SPEED:
		layerSize[1] = 30;
		break;
	case TRAINING_TYPE_STEER:
		layerSize[1] = 10;
		break;
	case TRAINING_TYPE_GEAR:
		layerSize[0] = 1;
		break;
	default:
		break;
	}

	// Learning rate - beta
	// momentum - alpha
	// Threshold - thresh (value of target mse, training stops once it is achieved)
	double beta = 0.0001, alpha = 0.5, Thresh =  0.2;

	std::vector<Car*> trainingData;
	bool success = loadCarData(filename, trainingData);

	if (!success)
		throw("Failed");

	std::cout << std::endl <<  "Now training the network...." << std::endl;

	CBackProp* bp = new CBackProp(numLayers, layerSize, beta, alpha);

	unsigned int size = trainingData.size();

	Car* car = NULL;
	int counter = 0;
	int iterations = 0;
	double error = 0;
	double percent = 0;

	bool trained = false;
	while (!trained)
	{
		int nrOfCorrect = 0;
		int total = 0;

		auto it_end = trainingData.cend();
		for (auto it = trainingData.cbegin(); it != it_end; it++)
		{
			int distance = std::distance(trainingData.cbegin(), it);
			car = (*it);

			double data[] = {
				car->speed, car->angle, car->distR, car->distFR, car->distFFR, 
				car->distF, car->distL, car->distFL, car->distFFL, car->clutch};

			double outputVal = 0.5;
			switch (type)
			{
			case TRAINING_TYPE_FULL:
				break;
			case TRAINING_TYPE_SPEED:
				if (car->accel > 0.0)
					outputVal = car->accel;
				else if(car->brake > 0.0)
					outputVal = car->brake;

				outputVal = 0.5 * (outputVal + 1.0);
				break;
			case TRAINING_TYPE_STEER:
				outputVal = 0.5 * (car->steer + 1.0);
				break;
			case TRAINING_TYPE_GEAR:
				outputVal = 0.5 * (car->gear + 1.0);
				break;
			default:
				break;
			}

			bp->bpgt(data, &outputVal);
		}
		double totalValue = 0.0;

		for (auto it = trainingData.cbegin(); it != it_end; it++)
		{
			int distance = std::distance(trainingData.cbegin(), it);
			car = (*it);

			double outputVal = 0.0;
			if (car->accel > 0.0)
				outputVal = car->accel;
			else if(car->brake > 0.0)
				outputVal = car->brake;

			double data[] = {
				car->speed, car->angle, car->distR, car->distFR, car->distFFR, 
				car->distF, car->distL, car->distFL, car->distFFL, car->clutch};

			bp->ffwd(data);

			double net_Out = bp->Out(0);

			double value = abs(net_Out - outputVal);
			totalValue += value;

			if (value < Thresh)
				nrOfCorrect++;

			total++;
		}
		error = totalValue / trainingData.size();
		percent = (double)((double)nrOfCorrect / (double)total);

		trained = error < Thresh || (1 - percent) < Thresh;

		counter++;
		if (trained || counter >= 1000)
		{
			iterations += counter;

			std::cout << std::endl
				<< "Iteration " << iterations << std::endl
				<< "Average Error " << error << std::endl
				<< "Percentage correct "<< percent << std::endl;

			counter = 0;
		}
	}

	std::cout << std::endl 
		<< "Iteration " << iterations << std::endl
		<< "Average Error " << error << std::endl
		<< "Percentage correct "<< percent << std::endl;


	switch (type)
	{
	case TRAINING_TYPE_SPEED:
		bp->WriteWeights("Bp_Speed.txt");
		break;
	case TRAINING_TYPE_STEER:
		bp->WriteWeights("Bp_Steer.txt");
		break;
	case TRAINING_TYPE_GEAR:
		bp->WriteWeights("Bp_Gear.txt");
		break;
	case TRAINING_TYPE_FULL:
	default:
		bp->WriteWeights("Bp_Full.txt");
		break;
	}

	std::cout << "Press Enter to quit" << std::endl;
	std::cin.ignore(1);

}

int main()
{
	
	//std::cout << "Running Speed Training (Accel & Brake)" << std::endl;
	//Train(TRAINING_TYPE_SPEED, "E-Track_3.txt");

	//std::cout << "Running Steering Training" << std::endl;
	//TrainBackProp(TRAINING_TYPE_STEER, "E-Track-4.txt");

	//std::cout << "Running Gear Training" << std::endl;
	//TrainBackProp(TRAINING_TYPE_GEAR, "E-Track_3.txt");
	
	std::vector<Car*> trainingData;
	//loadCarData("TrainingData/Road_E-Track 1.txt", trainingData);
	//loadCarData("TrainingData/Road_E-Track 2.txt", trainingData);
	//loadCarData("TrainingData/Road_Corkscrew.txt", trainingData);
	//loadCarData("TrainingData/Road_Olethros-Road-1.txt", trainingData);
	//loadCarData("TrainingData/Oval_Michigan-Speedway.txt", trainingData);
	//loadCarData("TrainingData/Dirt_Mixed-1.txt", trainingData);
	//loadCarData("TrainingData/Dirt_Dirt-2.txt", trainingData);
	//loadCarData("TrainingData/Road_Wheel 2.txt", trainingData);

	loadCarData("TrainingData/Road_Alpine 1.txt", trainingData);
	loadCarData("TrainingData/Road_Wheel 2.txt", trainingData);

	TrainAnnController(TRAINING_TYPE_FULL, trainingData);

	//TrainAnnController(TRAINING_TYPE_STEER, trainingData);
	//TrainAnnController(TRAINING_TYPE_SPEED, trainingData);
	//TrainAnnController(TRAINING_TYPE_GEAR, trainingData);

	std::cout << "Press Enter to quit" << std::endl;
	std::cin.ignore(1);

	return 0;
}