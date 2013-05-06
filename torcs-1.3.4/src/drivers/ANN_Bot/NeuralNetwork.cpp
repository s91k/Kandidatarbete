#include "NeuralNetwork.h"

NeuralNetwork::NeuralNetwork( int nInputs, int nOutputs, int nHiddenLayers, int nNodesInHiddenLayers )
{
	this->zNInputs = nInputs;
	this->zNOutputs = nOutputs;
	this->zNLayers = nHiddenLayers + 2;
	this->zNHiddenNodesPerLayer = nNodesInHiddenLayers;

	this->Init();
}

void NeuralNetwork::Init()
{
	this->zInputLayer = NULL;
	this->zOutputLayer = NULL;

	//this->zLearningRate = 0.000000000001f;
	this->zLearningRate = 0.000000001f;
	this->zMomentum = 0.5f;

	//error check
	if(this->zNLayers < 2)
		return;

	//clear out the layers, in Case you're restarting the net
	zLayers.clear();

	//input layer
	AddLayer(this->zNInputs, 1, NN_INPUT);

	if (this->zNLayers > 2)
	{
		//First Hidden Layer connect back to inputs
		AddLayer(this->zNHiddenNodesPerLayer, this->zNInputs, NN_HIDDEN);

		//any other hidden layers connect to other hidden outputs
		//-3 since the first layer was the inputs,
		//the second (connected to inputs) was initialized above,
		//and the last one (connect to outputs) will be initialized below
		for (int i = 0; i < this->zNLayers - 3; i++)
			AddLayer(this->zNHiddenNodesPerLayer, this->zNHiddenNodesPerLayer, NN_HIDDEN);

		//Output layer connects to hidden
		AddLayer(this->zNOutputs, this->zNHiddenNodesPerLayer, NN_OUTPUT);
	}
	else
	{
		//output layer connects to inputs
		AddLayer(this->zNOutputs, this->zNInputs, NN_OUTPUT);
	}

	this->zInputLayer = &this->zLayers[0];
	this->zOutputLayer = &this->zLayers[this->zNLayers - 1];
}

void NeuralNetwork::AddLayer( int nNeurons, int nInputs, int type )
{
	this->zLayers.push_back(NeuralLayer(nNeurons, nInputs, type));
}

void NeuralNetwork::Train( std::vector<float>& nInputs, std::vector<float>& nExpectedOutput )
{
	this->SetInputs(nInputs);
	this->Propagate();
	this->FindError(nExpectedOutput);
	this->BackPropagate();
}

void NeuralNetwork::Use( std::vector<float>& nInputs, std::vector<float>& nOutputs )
{
	this->SetInputs(nInputs);
	this->Propagate();
	nOutputs.clear();

	this->zOutputLayer->GetOutput(nOutputs);
}

void NeuralNetwork::SetInputs( std::vector<float>& inputs )
{
	this->zInputLayer->SetInput(inputs);
}

void NeuralNetwork::FindError( std::vector<float>& expectedOutput )
{
	this->zError = this->zOutputLayer->CalculateError(expectedOutput);
}

void NeuralNetwork::Propagate()
{
	for (int i = 0; i < this->zNLayers - 1; i++)
	{
		this->zLayers[i].Propagate(this->zLayers[i + 1]);
	}
}

void NeuralNetwork::BackPropagate()
{
	//BackPropagate the error
	//for (int i = this->zNLayers - 1; i > 0; i--)
	//{
	//	this->zLayers[i].BackPropagate(this->zLayers[i - 1]);
	//}

	for (int i = this->zNLayers - 1; i > 0; i--)
	{
		this->zLayers[i - 1].BackPropagate(this->zLayers[i]);
	}

	//Adjust the weights
	for (int i =  0; i < this->zNLayers - 1; i++)
	{
		this->zLayers[i + 1].AdjustWeights(this->zLayers[i], zLearningRate, zMomentum);
	}
}

void NeuralNetwork::WriteWeights()
{
	FILE* pFile;
	fopen_s(&pFile, "NNWeightData.txt", "w");
	if (pFile == NULL)
		return;

	for (int i = 0; i < this->zNLayers; i++)
	{
		int numNeurons = this->zLayers[i].zNeurons.size();

		for (int j = 0; j < numNeurons; j++)
		{
			int numWeights = this->zLayers[i].zNeurons[j]->zWeights.size();
			for (int k = 0; k < numWeights; k++)
				fprintf(pFile, "%f \n", this->zLayers[i].zNeurons[j]->zWeights[k]);
		}
	}
	fclose(pFile);
}

void NeuralNetwork::ReadWeights()
{
	FILE* pFile;
	fopen_s(&pFile, "NNWeightData.txt", "r");
	if (pFile == NULL)
		return;

	for(int i = 0; i < this->zNLayers; i++)
	{
		int numNeurons = this->zLayers[i].zNeurons.size();
		for(int j = 0; j < numNeurons; j++)
		{
			int numWeights = this->zLayers[i].zNeurons[j]->zWeights.size();
			for(int k = 0; k < numWeights; k++)
				fscanf(pFile,"%f ",&this->zLayers[i].zNeurons[j]->zWeights[k]);
		}
	}
	fclose(pFile);
}