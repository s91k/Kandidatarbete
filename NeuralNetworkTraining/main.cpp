#include "AnnAIController.h"
#include <iostream>

int main()
{

	_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	AnnAIController* controller = new AnnAIController();
	
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

	controller->LoadTrainingData(texfiles[input]);
	
	while(!controller->RunTraining())
	{

	}

	delete controller;

	std::cout << "Press Enter to quit" << std::endl;
	std::cin.ignore(1);

	return 0;
}