#include "AnnAIController.h"
#include <iostream>

int main()
{
	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	AnnAIController* controllerFull = new AnnAIController();
	AnnAIController* controllerSpeed = new AnnAIController(TRAINING_TYPE_SPEED);
	AnnAIController* controllerSteering = new AnnAIController(TRAINING_TYPE_STEER);
	AnnAIController* controllerGear = new AnnAIController(TRAINING_TYPE_GEAR);

	std::cout << "Choose a file number between 0 - 7" << std::endl;

	int input = 0;

	std::cin >> input;
	std::cin.ignore(1);
	std::string texfiles[] = 
	{
		"berniw_Recorder 1 recording.txt",
		"berniw_Recorder 2 recording.txt",
		"berniw_Recorder 3 recording.txt",
		"berniw_Recorder 4 recording.txt",
		"berniw_Recorder 5 recording.txt",
		"berniw_Recorder 6 recording.txt",
		"berniw_Recorder 7 recording.txt",
		"berniw_Recorder 8 recording.txt",
	};
	if (input < 0)
		input = 0;

	else if (input > 7)
		input = 7;

	float error = 0.0f;
	bool Full_Training = false;
	//Run Training
	if (Full_Training)
	{
		controllerFull->LoadTrainingData(texfiles[input]);
		controllerFull->RunTraining();

		error = controllerFull->GetFinalError();
		std::cout << "Final Error for Full Training = " << error << std::endl;
	}
	else
	{
		std::cout << "Running speed training (Accel & Brake)" << std::endl;
		controllerSpeed->LoadTrainingData(texfiles[input]);
		controllerSpeed->RunTraining();

		std::cout << "Running Steering training" << std::endl;
		controllerSteering->LoadTrainingData(texfiles[input]);
		controllerSteering->RunTraining();

		std::cout << "Running Gear training" << std::endl;
		controllerGear->LoadTrainingData(texfiles[input]);
		controllerGear->RunTraining();

		error = controllerSpeed->GetFinalError();
		std::cout << "Final Error for Speed Training = " << error << std::endl;

		error = controllerSteering->GetFinalError();
		std::cout << "Final Error for Steering Training = " << error << std::endl;

		error = controllerGear->GetFinalError();
		std::cout << "Final Error for Gear Training = " << error << std::endl;
	}

	

	//delete controllerFull;
	delete controllerSpeed;
	delete controllerSteering;
	delete controllerGear;

	std::cout << "Press Enter to quit" << std::endl;
	std::cin.ignore(1);

	return 0;
}