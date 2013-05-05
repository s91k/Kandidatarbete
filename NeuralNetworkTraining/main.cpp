#include "AnnAIController.h"
#include <iostream>

int main()
{
	AnnAIController* controller = new AnnAIController();
	
	controller->LoadTrainingData("berniw_Recorder 1 recording.txt");
	
	while(!controller->Update())
	{

	}
	std::cout << "Press Enter to quit" << std::endl;
	std::cin.ignore(1);

	return 0;
}