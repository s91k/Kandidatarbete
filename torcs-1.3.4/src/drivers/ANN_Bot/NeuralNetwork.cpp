#include "NeuralNetwork.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
	this->zLayerType = type;
	for (int i = 0; i < nNeurons; i++)
	{
		Neuron* newNeuron = new Neuron();

		for (int j = 0; j < nInputs; j++)
		{
			float valueOne = rand() / RAND_MAX;
			float valueTwo = rand() / RAND_MAX;
			
			//Random value between 1 & -1
			newNeuron->zWeights.push_back(valueOne - valueTwo);
			newNeuron->zLastDelta.push_back(0.0f);
		}

		//Initial Values
		newNeuron->zOutput = 0.0f;
		newNeuron->zError = 99999.9f;

		this->zNeurons.push_back(newNeuron);
	}
}

void NeuralLayer::Propogate( NeuralLayer& nexLayer )
{

}

void NeuralLayer::BackPropogate( NeuralLayer& nexLayer )
{

}

void NeuralLayer::AdjustWeights( NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/ )
{

}


void NeuralNetwork::AddLayer( int nNeurons, int nInputs, int type )
{
	this->zLayers.push_back(NeuralLayer(nNeurons, nInputs, type));
}

NeuralNetwork::NeuralNetwork( int nInputs, int nOutputs, int nHiddenLayers, int nNodesInHiddenLayers )
{
	this->zNInputs = nInputs;
	this->zNOutputs = nOutputs;
	this->zNLayers = nHiddenLayers + 2;
	this->zNHiddenNodesPerLayer = nNodesInHiddenLayers;

	this->Init();
}

void NeuralNetwork::Train( std::vector<float>& nInputs, std::vector<float>& nOutputs )
{

}

void NeuralNetwork::WriteWeights()
{

}

void NeuralNetwork::ReadWeights()
{

}

void NeuralNetwork::Init()
{
	this->zInputLayer = NULL;
	this->zOutputLayer = NULL;
	
	this->zLearningRate = 0.1f;
	this->zMomentum = 0.9f;

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